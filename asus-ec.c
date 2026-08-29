// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Embedded-controller driver for ASUS laptops whose EC exposes the
 * "HealthyTable" I/O-port protocol used by Windows' AsusSAIO.sys /
 * AsusWinIO64.dll. The protocol was reverse engineered from a
 * userspace proof-of-concept by PisonJay (HealthyTable /
 * AsusFanControl); see the driver submission thread for links.
 *
 * Currently implements manual fan-duty control (hwmon pwm1/pwm2) on
 * the boards listed in asus_ec_dmi_table below. The driver is
 * structured as a general ASUS-EC driver rather than a fan-only one:
 * asus_ec_quirks carries a capability flag per matched board, so a
 * future feature reachable through this same EC transport (e.g. TDP
 * control) can be added as another flag and another DMI entry instead
 * of forking a second driver for the same I/O ports.
 *
 * BACKGROUND (ASUS V16 V3607VM): this board uses the legacy ATK-style
 * asus-nb-wmi dispatch ("Detected ATK, not ASUSWMI, use DSTS" in
 * dmesg), on which the modern ASUS WMI fan-control SET method is not
 * reachable. An EC mailbox reachable through ACPI does exist on this
 * class of board, but was found to silently revert an applied duty
 * back to automatic within 5-30s without a continuous refresh;
 * HealthyTable held a requested duty steady with zero refreshes in
 * every trial run against it.
 *
 * TRANSPORT: direct x86 I/O port access (0x25c data / 0x25d status),
 * not ACPI. This bypasses any ACPI-side EC mutex entirely, so all
 * serialization responsibility falls on this driver's own per-device
 * mutex - there is no firmware-side safety net backing this transport.
 *
 * FAN INDEX MAPPING: HealthyTable indexes idx 0 = CPU, idx 1 = GPU;
 * see hwmon_channel_to_idx below.
 *
 * SAFETY: this driver must never load on hardware that has not been
 * explicitly verified to expose this EC protocol. The DMI match table
 * below is the only gate - do not replace it with a heuristic or a
 * broader (e.g. vendor-only) match without per-board verification.
 * Raw I/O port access to an unverified EC can corrupt EC state or hang
 * the board, so the DMI gate is load-bearing, not a formality.
 *
 * No kernel-side auto-revert timer by design - userspace is
 * responsible for returning fans to auto (via pwm_enable=2) when it's
 * done; the kernel does not babysit it.
 */

#define DRIVER_NAME "asus_ec"
#define pr_fmt(fmt) DRIVER_NAME ": " fmt

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/hwmon.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/printk.h>

#if !defined(CONFIG_X86)
#error "asus-ec requires x86 I/O port access (inb/outb)"
#endif

/*
 * HealthyTable I/O ports and protocol constants, ported from the
 * reverse-engineered userspace proof-of-concept referenced above.
 */
#define ASUS_EC_DATA_PORT   0x25c
#define ASUS_EC_STATUS_PORT 0x25d
#define ASUS_EC_PORT_COUNT  2

#define ASUS_EC_STATUS_OBF 0x01
#define ASUS_EC_STATUS_IBF 0x02

#define ASUS_EC_CMD_PREFIX        0xff
#define ASUS_EC_CMD_HEALTHY_PROBE 0xbb
#define ASUS_EC_CMD_HEALTHY_TABLE 0xdd

/*
 * Query byte for ASUS_EC_CMD_HEALTHY_PROBE. Opaque - taken as-is from
 * PisonJay's PoC, which sends this literal without documenting what it
 * selects. Distinct from the ASUS_HT_ID_* selectors below, which are
 * only meaningful with ASUS_EC_CMD_HEALTHY_TABLE, not this probe.
 */
#define ASUS_HT_PROBE_QUERY 0x50

#define ASUS_HT_READ  0x02
#define ASUS_HT_WRITE 0x82

#define ASUS_HT_ID_TEST_MODE 0x31
#define ASUS_HT_ID_FAN_INDEX 0x32
#define ASUS_HT_ID_PWM_DUTY  0x35

#define ASUS_MAX_PACKET_DATA 8

/*
 * Overall poll timeout, and the delay between each status-port sample.
 * IBF/OBF transitions normally happen in the microsecond range, well
 * inside ASUS_EC_SPIN_US below; ASUS_EC_TIMEOUT_US is only reached on
 * a genuinely wedged EC.
 */
#define ASUS_EC_TIMEOUT_US 500000
#define ASUS_EC_POLL_DELAY_US 100

/*
 * Busy-spin budget for the poll loop below. msleep()'s ~1ms+ jiffy
 * granularity is too coarse for the common case, where the EC
 * responds within a few ASUS_EC_POLL_DELAY_US ticks, so the wait
 * starts as a udelay() busy-spin. Past this budget, the loop switches
 * to usleep_range() for the remainder of ASUS_EC_TIMEOUT_US instead of
 * continuing to spin - the per-device lock is a plain mutex, not a
 * spinlock, so sleeping while holding it is fine, and it keeps a
 * wedged EC from pinning a CPU core for the full 500ms worst case.
 */
#define ASUS_EC_SPIN_US 1000

/*
 * Per-board capability flags, reached via dmi_system_id.driver_data.
 * Only one capability exists today; this exists as a struct (not a
 * bare bool) so a second EC-reachable feature (e.g. TDP control) can
 * be added as another field without another driver or another DMI
 * table walk.
 */
struct asus_ec_quirks {
	bool has_fan_control;
};

static const struct asus_ec_quirks asus_ec_v3607vm_quirks = {
	.has_fan_control = true,
};

/*
 * DMI match table. Each entry's .driver_data points at the capability
 * flags for that board (struct asus_ec_quirks above). DMI_MATCH is
 * exact-substring, case-sensitive - copy verbatim, don't paraphrase or
 * guess formatting when adding a board.
 *
 * The BIOS version is deliberately NOT part of any entry's match: a
 * future BIOS update should not silently unbind this driver. If the
 * HealthyTable protocol ever changes in a way that breaks a board, the
 * probe-time sanity check (see asus_ec_probe()) is the backstop, not
 * a BIOS-version pin here. Each entry's comment below records the
 * BIOS version(s) it was actually tested against, for reference only.
 */
static const struct dmi_system_id asus_ec_dmi_table[] = {
	{
		/* Tested against BIOS V3607VM.310. */
		.ident = "ASUS V16 V3607VM",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "ASUSTeK COMPUTER INC."),
			DMI_MATCH(DMI_PRODUCT_NAME, "ASUS V16 V3607VM_V3607VM"),
			DMI_MATCH(DMI_BOARD_NAME, "V3607VM"),
		},
		.driver_data = (void *)&asus_ec_v3607vm_quirks,
	},
	{ }
};

/*
 * MODULE_DEVICE_TABLE(dmi, ...) is the autoload mechanism here: depmod
 * generates a DMI modalias from this table, so udev loads the module
 * automatically on a matching board without needing a separate
 * MODULE_ALIAS() or any manual modprobe/insmod at boot. No broader
 * alias is added on purpose - autoload is meant to stay scoped to
 * exactly the boards listed above.
 */
MODULE_DEVICE_TABLE(dmi, asus_ec_dmi_table);

/*
 * Per-instance driver state, reached via platform_set_drvdata() /
 * dev_get_drvdata() rather than file-scope globals - this driver only
 * ever has one instance in practice (DMI-gated to specific boards),
 * but tying state to the device instance rather than the module is
 * the correct pattern regardless, and costs nothing here.
 */
struct asus_ec_data {
	/*
	 * Serializes ALL HealthyTable I/O port transactions, and
	 * protects every other field below. This matters more here
	 * than it would on an ACPI transport: the protocol is
	 * inherently stateful (select_fan() sets a "current fan"
	 * register that subsequent test-mode/duty operations act on),
	 * and there is no firmware-side mutex backing this up - if two
	 * callers' port sequences interleave, a duty write can land on
	 * the wrong fan.
	 */
	struct mutex lock;

	bool override_active[2];

	/*
	 * Last-requested duty per fan index (0=CPU, 1=GPU - see fan
	 * index mapping note at the top of this file), 0-255. Only
	 * pushed to hardware while override_active[idx] is true.
	 *
	 * Initialized to -1, not 0: 0 is a valid duty value (fan fully
	 * off under override), so it can't double as "nothing
	 * requested yet". asus_ec_fan_hwmon_read() treats any negative
	 * value as "report 0" - -1 only ever shows up before the first
	 * write, which is indistinguishable from 0% to a reader either
	 * way, so no separate "unset" case is needed there.
	 */
	int pwm_value[2];

	/*
	 * asus_ec_ht_probe()'s 0xBB handshake reports a fixed interface
	 * version that the EC firmware does not change at runtime -
	 * cache it after the first successful probe and skip the
	 * handshake on every later transaction. Guarded by lock above,
	 * same as everything else in this struct. Invalidated (see
	 * asus_ec_ht_read_u8()/asus_ec_ht_write_u8()) whenever a
	 * transaction fails, so a real EC hiccup causes a fresh
	 * re-probe rather than every later call failing silently on a
	 * stale assumption.
	 */
	bool ht_probed;
	u8 ht_version;
};

/*
 * hwmon channel -> fan index mapping. pwm1 (hwmon channel 0) = CPU;
 * pwm2 (hwmon channel 1) = GPU. HealthyTable's own indexing is already
 * idx 0 = CPU, idx 1 = GPU, so this is the identity map.
 */
static const int hwmon_channel_to_idx[2] = { 0, 1 };

/* ---- low-level HealthyTable I/O port transport ---- */

/*
 * Shared core for the three status-port waits below. Polls until
 * (status & mask) reaches want_set, or ASUS_EC_TIMEOUT_US elapses; see
 * ASUS_EC_SPIN_US above for the spin/sleep split. If drain is true,
 * any byte that shows up in the data port while waiting is read and
 * discarded, used to flush stale EC output before a new transaction.
 * label/phase are only used to build the timeout log message; phase
 * may be NULL.
 *
 * Assumes the caller holds data->lock.
 */
static int asus_ec_poll_status(u8 mask, bool want_set, bool drain,
			       const char *label, const char *phase)
{
	ktime_t deadline = ktime_add_us(ktime_get(), ASUS_EC_TIMEOUT_US);
	ktime_t spin_deadline = ktime_add_us(ktime_get(), ASUS_EC_SPIN_US);

	while (ktime_before(ktime_get(), deadline)) {
		bool is_set = inb(ASUS_EC_STATUS_PORT) & mask;

		if (is_set == want_set)
			return 0;

		if (drain)
			(void)inb(ASUS_EC_DATA_PORT);

		if (ktime_before(ktime_get(), spin_deadline))
			udelay(ASUS_EC_POLL_DELAY_US);
		else
			usleep_range(ASUS_EC_POLL_DELAY_US, ASUS_EC_POLL_DELAY_US * 2);
	}

	pr_err("timeout waiting for %s%s%s\n", label,
	       phase ? " during " : "", phase ? phase : "");
	return -ETIMEDOUT;
}

static int asus_ec_wait_ibf_clear(const char *phase)
{
	return asus_ec_poll_status(ASUS_EC_STATUS_IBF, false, false,
				    "input buffer clear", phase);
}

static int asus_ec_wait_obf_set(const char *phase)
{
	return asus_ec_poll_status(ASUS_EC_STATUS_OBF, true, false,
				    "output buffer full", phase);
}

static int asus_ec_drain_output(void)
{
	return asus_ec_poll_status(ASUS_EC_STATUS_OBF, false, true,
				    "pending EC output to drain", NULL);
}

static int asus_ec_write_command_byte(u8 value)
{
	int ret = asus_ec_wait_ibf_clear("command write");

	if (ret)
		return ret;
	outb(value, ASUS_EC_STATUS_PORT);
	return 0;
}

static int asus_ec_write_data_byte(u8 value)
{
	int ret = asus_ec_wait_ibf_clear("data write");

	if (ret)
		return ret;
	outb(value, ASUS_EC_DATA_PORT);
	return 0;
}

/*
 * Core transaction primitive, ported 1:1 from the PoC's ec_xfer().
 * Assumes the caller holds data->lock. read_value must be non-NULL
 * when read_back is true - the transaction always reads back exactly
 * one byte, there is no variable-length read path to bounds-check.
 */
static int asus_ec_xfer(u8 command, const u8 *data, size_t data_len,
			bool read_back, u8 *read_value)
{
	int ret;
	size_t i;

	if (data_len > ASUS_MAX_PACKET_DATA)
		return -EINVAL;
	if (read_back && !read_value)
		return -EINVAL;

	ret = asus_ec_drain_output();
	if (ret)
		return ret;

	ret = asus_ec_write_command_byte(ASUS_EC_CMD_PREFIX);
	if (ret)
		return ret;

	ret = asus_ec_write_command_byte(command);
	if (ret)
		return ret;

	for (i = 0; i < data_len; i++) {
		ret = asus_ec_write_data_byte(data[i]);
		if (ret)
			return ret;
	}

	ret = asus_ec_wait_ibf_clear("transaction completion");
	if (ret)
		return ret;

	if (read_back) {
		ret = asus_ec_wait_obf_set("readback");
		if (ret)
			return ret;
		*read_value = inb(ASUS_EC_DATA_PORT);
	}

	return 0;
}

/* ---- HealthyTable protocol layer ---- */

static int asus_ec_ht_probe(u8 *version)
{
	const u8 data[] = { ASUS_HT_PROBE_QUERY };
	int ret;

	ret = asus_ec_xfer(ASUS_EC_CMD_HEALTHY_PROBE, data, sizeof(data), true, version);
	if (ret)
		return ret;

	if (*version == 0) {
		pr_err("HealthyTable probe returned 0; interface may be absent\n");
		return -ENODEV;
	}

	return 0;
}

/* Assumes the caller holds data->lock. */
static int asus_ec_ht_ensure_probed(struct asus_ec_data *data)
{
	int ret;

	if (data->ht_probed)
		return 0;

	ret = asus_ec_ht_probe(&data->ht_version);
	if (ret)
		return ret;

	data->ht_probed = true;
	return 0;
}

static int asus_ec_ht_read_u8(struct asus_ec_data *data, u8 selector, u8 arg, u8 *value)
{
	const u8 payload[] = { ASUS_HT_READ, selector, arg };
	int ret;

	ret = asus_ec_ht_ensure_probed(data);
	if (ret)
		return ret;

	ret = asus_ec_xfer(ASUS_EC_CMD_HEALTHY_TABLE, payload, sizeof(payload), true, value);
	if (ret)
		data->ht_probed = false;

	return ret;
}

static int asus_ec_ht_write_u8(struct asus_ec_data *data, u8 selector, u8 value)
{
	const u8 payload[] = { ASUS_HT_WRITE, selector, value };
	int ret;

	ret = asus_ec_ht_ensure_probed(data);
	if (ret)
		return ret;

	ret = asus_ec_xfer(ASUS_EC_CMD_HEALTHY_TABLE, payload, sizeof(payload), false, NULL);
	if (ret)
		data->ht_probed = false;

	return ret;
}

/* Assumes the caller holds data->lock. */
static int asus_ec_select_fan_locked(struct asus_ec_data *data, int idx)
{
	return asus_ec_ht_write_u8(data, ASUS_HT_ID_FAN_INDEX, (u8)idx);
}

static int asus_ec_set_test_mode_locked(struct asus_ec_data *data, int idx, bool enable)
{
	int ret = asus_ec_select_fan_locked(data, idx);

	if (ret)
		return ret;
	return asus_ec_ht_write_u8(data, ASUS_HT_ID_TEST_MODE, enable ? 1 : 0);
}

static int asus_ec_ht_set_duty_locked(struct asus_ec_data *data, int idx, u8 duty)
{
	int ret = asus_ec_select_fan_locked(data, idx);

	if (ret)
		return ret;
	return asus_ec_ht_write_u8(data, ASUS_HT_ID_PWM_DUTY, duty);
}

/*
 * Public wrapper: enable/disable manual override on fan idx. Updates
 * override_active[] under the lock on every call. No kernel-side
 * auto-revert timer by design.
 */
static int asus_ec_fan_enable(struct asus_ec_data *data, int idx, bool enable)
{
	int ret;

	guard(mutex)(&data->lock);

	ret = asus_ec_set_test_mode_locked(data, idx, enable);
	if (ret == 0)
		data->override_active[idx] = enable;
	return ret;
}

/*
 * Locked variant, for callers (hwmon_write) that need to check
 * override_active[] and issue the duty write as a single atomic
 * critical section, avoiding a TOCTOU window where another writer
 * could disable the override between the check and the EC write.
 */
static int asus_ec_fan_set_duty_locked(struct asus_ec_data *data, int idx, int duty)
{
	int ret = asus_ec_ht_set_duty_locked(data, idx, (u8)duty);

	if (ret == 0)
		data->pwm_value[idx] = duty;
	return ret;
}

/*
 * Read-only helper: current test/manual mode for fan idx (0 = auto,
 * nonzero = manual). Used only for the probe()-time sanity check -
 * never as a hwmon-visible source of truth. hwmon reads are served
 * from override_active[]/pwm_value[] instead: EC-reported state is
 * not trusted as authoritative once the driver is tracking its own
 * view of it.
 */
static int asus_ec_read_test_mode(struct asus_ec_data *data, int idx)
{
	int ret;
	u8 mode;

	guard(mutex)(&data->lock);

	ret = asus_ec_select_fan_locked(data, idx);
	if (ret)
		return ret;

	ret = asus_ec_ht_read_u8(data, ASUS_HT_ID_TEST_MODE, 0, &mode);
	if (ret)
		return ret;

	return (int)mode;
}

/* ---- hwmon glue ---- */

static umode_t asus_ec_fan_hwmon_is_visible(const void *drvdata,
					    enum hwmon_sensor_types type,
					    u32 attr, int channel)
{
	if (type != hwmon_pwm)
		return 0;

	switch (attr) {
	case hwmon_pwm_input:
	case hwmon_pwm_enable:
		return 0644;
	default:
		return 0;
	}
}

static int asus_ec_fan_hwmon_read(struct device *dev,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel, long *val)
{
	struct asus_ec_data *data = dev_get_drvdata(dev);
	int idx;

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;
	if (channel < 0 || channel > 1)
		return -EINVAL;

	idx = hwmon_channel_to_idx[channel];

	guard(mutex)(&data->lock);

	switch (attr) {
	case hwmon_pwm_input:
		*val = data->pwm_value[idx] < 0 ? 0 : data->pwm_value[idx];
		return 0;
	case hwmon_pwm_enable:
		/*
		 * Standard hwmon pwm_enable encoding is 0=full-speed,
		 * 1=manual, 2=automatic. This EC only implements two
		 * states (test mode on/off) with no hardware equivalent
		 * of canonical 0 ("no PWM control, full speed") - it is
		 * always either under manual override or under its own
		 * automatic governor, never uncontrolled. So this driver
		 * reports/accepts only 1 and 2, and rejects 0 on write
		 * (see asus_ec_fan_hwmon_write() below) rather than
		 * pretending to support a state that doesn't exist here.
		 */
		*val = data->override_active[idx] ? 1 : 2;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int asus_ec_fan_hwmon_write(struct device *dev,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel, long val)
{
	struct asus_ec_data *data = dev_get_drvdata(dev);
	int idx;
	int ret;

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;
	if (channel < 0 || channel > 1)
		return -EINVAL;

	idx = hwmon_channel_to_idx[channel];

	switch (attr) {
	case hwmon_pwm_enable: {
		bool enable;

		switch (val) {
		case 1:
			enable = true;
			break;
		case 2:
			enable = false;
			break;
		default:
			/*
			 * Canonical 0 ("no PWM control, full speed") has
			 * no counterpart on this EC - see the comment in
			 * asus_ec_fan_hwmon_read() above.
			 */
			return -EINVAL;
		}

		ret = asus_ec_fan_enable(data, idx, enable);
		if (ret < 0)
			return ret;

		pr_debug("fan idx %d override %s\n", idx,
			 enable ? "ENABLED (manual)" : "disabled (auto)");
		return 0;
	}

	case hwmon_pwm_input: {
		guard(mutex)(&data->lock);

		if (val < 0 || val > 255)
			return -EINVAL;

		if (!data->override_active[idx]) {
			pr_debug("pwm write on idx %d while not in manual mode, rejecting\n",
				 idx);
			return -EOPNOTSUPP;
		}
		return asus_ec_fan_set_duty_locked(data, idx, (int)val);
	}

	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_ops asus_ec_fan_hwmon_ops = {
	.is_visible = asus_ec_fan_hwmon_is_visible,
	.read = asus_ec_fan_hwmon_read,
	.write = asus_ec_fan_hwmon_write,
};

static const struct hwmon_channel_info *asus_ec_fan_hwmon_info[] = {
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	NULL
};

static const struct hwmon_chip_info asus_ec_fan_hwmon_chip_info = {
	.ops = &asus_ec_fan_hwmon_ops,
	.info = asus_ec_fan_hwmon_info,
};

/* ---- platform_driver lifecycle ---- */

static int asus_ec_probe(struct platform_device *pdev)
{
	const struct asus_ec_quirks *quirks = pdev->dev.platform_data;
	struct asus_ec_data *data;
	struct device *hwdev;
	int i;
	int mode0, mode1;

	dev_info(&pdev->dev, "probe on matched DMI board\n");

	if (!quirks || !quirks->has_fan_control) {
		dev_info(&pdev->dev, "no supported EC feature on this board, nothing to bind\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->lock);
	data->pwm_value[0] = -1;
	data->pwm_value[1] = -1;
	platform_set_drvdata(pdev, data);

	if (!devm_request_region(&pdev->dev, ASUS_EC_DATA_PORT, ASUS_EC_PORT_COUNT, DRIVER_NAME)) {
		dev_err(&pdev->dev, "could not reserve I/O ports 0x%x-0x%x (in use by another driver?)\n",
			ASUS_EC_DATA_PORT, ASUS_EC_DATA_PORT + ASUS_EC_PORT_COUNT - 1);
		return -EBUSY;
	}

	/*
	 * Blind force-revert on both fans BEFORE anything else, in case a
	 * previous session left override active. Uses the locked-internal
	 * path directly (not asus_ec_fan_enable()) purely to avoid
	 * depending on override_active[] being meaningful yet; behavior is
	 * otherwise identical. Failure here is logged but non-fatal.
	 */
	scoped_guard(mutex, &data->lock) {
		for (i = 0; i < 2; i++) {
			int ret = asus_ec_set_test_mode_locked(data, i, false);

			if (ret < 0)
				dev_warn(&pdev->dev, "startup force-revert (fan %d) failed (non-fatal)\n",
					 i);
			data->override_active[i] = false;
		}
	}

	/*
	 * Sanity probe: reading test-mode on both known fan indices must
	 * succeed, or we bail out - this is the fail-safe for boards that
	 * pass the DMI match table but don't actually speak this protocol
	 * (e.g. a future BIOS/EC revision).
	 */
	mode0 = asus_ec_read_test_mode(data, 0);
	if (mode0 < 0) {
		dev_err(&pdev->dev, "test-mode sanity check (fan 0) failed, not binding\n");
		return -ENODEV;
	}

	mode1 = asus_ec_read_test_mode(data, 1);
	if (mode1 < 0) {
		dev_err(&pdev->dev, "test-mode sanity check (fan 1) failed, not binding\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "test-mode(0)=%d test-mode(1)=%d (raw EC test-mode; sanity check OK, expect 0 = auto)\n",
		 mode0, mode1);

	hwdev = devm_hwmon_device_register_with_info(&pdev->dev, "asus_ec_fan", data,
						     &asus_ec_fan_hwmon_chip_info, NULL);
	if (IS_ERR(hwdev)) {
		dev_err(&pdev->dev, "hwmon registration failed: %ld\n", PTR_ERR(hwdev));
		return PTR_ERR(hwdev);
	}

	dev_info(&pdev->dev, "hwmon registered (pwm1=CPU idx0, pwm2=GPU idx1)\n");

	return 0;
}

static void asus_ec_remove(struct platform_device *pdev)
{
	struct asus_ec_data *data = platform_get_drvdata(pdev);
	int i;

	dev_info(&pdev->dev, "remove, reverting both fans to auto\n");

	/*
	 * The I/O port region and data->lock are devm-managed and released
	 * automatically after this function returns - the revert below
	 * must happen here, while the ports are still ours.
	 */
	scoped_guard(mutex, &data->lock) {
		for (i = 0; i < 2; i++) {
			int ret = asus_ec_set_test_mode_locked(data, i, false);

			if (ret < 0) {
				dev_err(&pdev->dev, "remove: revert(%d) FAILED, fan %d may be stuck manual!\n",
					i, i);
			}
			data->override_active[i] = false;
		}
	}
}

static int asus_ec_pm_suspend(struct device *dev)
{
	struct asus_ec_data *data = dev_get_drvdata(dev);
	int i;

	dev_info(dev, "suspend, force-reverting both fans\n");

	scoped_guard(mutex, &data->lock) {
		for (i = 0; i < 2; i++) {
			asus_ec_set_test_mode_locked(data, i, false);
			data->override_active[i] = false;
		}
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(asus_ec_pm_ops, asus_ec_pm_suspend, NULL);

static void asus_ec_shutdown(struct platform_device *pdev)
{
	struct asus_ec_data *data = platform_get_drvdata(pdev);
	int i;

	dev_info(&pdev->dev, "shutdown, force-reverting both fans\n");

	scoped_guard(mutex, &data->lock) {
		for (i = 0; i < 2; i++) {
			asus_ec_set_test_mode_locked(data, i, false);
			data->override_active[i] = false;
		}
	}
}

static struct platform_driver asus_ec_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.pm = pm_ptr(&asus_ec_pm_ops),
	},
	.probe = asus_ec_probe,
	.remove = asus_ec_remove,
	.shutdown = asus_ec_shutdown,
};

static struct platform_device *asus_ec_pdev;

static int __init asus_ec_init(void)
{
	const struct dmi_system_id *match;
	int ret;

	match = dmi_first_match(asus_ec_dmi_table);
	if (!match) {
		pr_info("no matching DMI board, not loading\n");
		return -ENODEV;
	}

	/*
	 * The matched quirks struct is handed to probe() via
	 * platform_data rather than a global - platform_device_register()
	 * has no driver-data slot of its own before probe() runs, and
	 * platform_data is exactly the mechanism meant for this.
	 */
	asus_ec_pdev = platform_device_register_data(NULL, DRIVER_NAME, -1,
						     match->driver_data,
						     sizeof(struct asus_ec_quirks));
	if (IS_ERR(asus_ec_pdev)) {
		pr_err("failed to register platform device\n");
		return PTR_ERR(asus_ec_pdev);
	}

	ret = platform_driver_register(&asus_ec_driver);
	if (ret) {
		pr_err("failed to register platform driver\n");
		platform_device_unregister(asus_ec_pdev);
		return ret;
	}

	return 0;
}

static void __exit asus_ec_exit(void)
{
	platform_driver_unregister(&asus_ec_driver);
	platform_device_unregister(asus_ec_pdev);
}

module_init(asus_ec_init);
module_exit(asus_ec_exit);

MODULE_AUTHOR("Niko Iwamura");
MODULE_DESCRIPTION("ASUS EC control via the HealthyTable I/O-port protocol - fan hwmon pwm1/pwm2");
MODULE_LICENSE("GPL");
