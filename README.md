# asus-ec

Linux kernel driver for ASUS laptops with Embedded Controllers (EC) that expose the "HealthyTable" I/O-port protocol.

## Overview

This driver implements manual fan-duty control through direct x86 I/O port access, bypassing ACPI. It was created for ASUS laptops where the modern ASUS WMI fan-control methods are not reachable, but an EC mailbox accessible through I/O ports exists.

**Currently implements:** Manual fan control (hwmon pwm1/pwm2) for CPU and GPU fans on supported boards.

## Supported Hardware

The driver uses a DMI match table to ensure it only loads on verified hardware. Currently supported:

- **ASUS V16 V3607VM** (tested with BIOS V3607VM.310)

> **Important:** The DMI match table is a safety gate. Do not attempt to load this driver on unsupported hardware - raw I/O port access to an unverified EC can corrupt EC state or hang the board.

## How It Works

### Transport Layer
- Direct x86 I/O port access (ports 0x25c data / 0x25d status)
- Bypasses ACPI-side EC mutex entirely
- All serialization handled by the driver's per-device mutex

### HealthyTable Protocol
Reverse-engineered from a userspace proof-of-concept by PisonJay (HealthyTable/AsusFanControl). The protocol uses:
- Command prefix: `0xff`
- Probe command: `0xbb`
- Table command: `0xdd`

### Fan Index Mapping
- **pwm1 (hwmon channel 0)** = CPU fan (index 0)
- **pwm2 (hwmon channel 1)** = GPU fan (index 1)

## Installation

### Building as a Kernel Module

```bash
# Copy the driver to your kernel source tree
cp asus-ec.c /path/to/kernel/drivers/platform/x86/

# Add to Makefile in drivers/platform/x86/Makefile:
# obj-m += asus-ec.o

# Build the module
make -C /lib/modules/$(uname -r)/build M=/path/to/kernel/drivers/platform/x86/ modules
```

### Loading the Module

The driver will auto-load on supported hardware via udev modalias. To load manually:

```bash
sudo modprobe asus_ec
```

Check if loaded:
```bash
lsmod | grep asus_ec
dmesg | grep asus_ec
```

## Usage

The driver registers with the hwmon subsystem, providing standard PWM fan control interfaces.

### Sysfs Interface

Once loaded, the driver creates hwmon entries:

```bash
# Check available hwmon devices
ls /sys/class/hwmon/

# Find the asus_ec_fan device
grep -l "asus_ec_fan" /sys/class/hwmon/*/name

# Example path (replace hwmonX with actual number):
# /sys/class/hwmon/hwmonX/
```

### Fan Control

#### Enable Manual Mode (pwm_enable)

Standard hwmon pwm_enable encoding:
- `1` = Manual mode (override active)
- `2` = Automatic mode (EC controls fan)
- `0` = Not supported (rejected with EINVAL)

```bash
# Enable manual control for CPU fan (pwm1)
echo 1 | sudo tee /sys/class/hwmon/hwmonX/pwm1_enable

# Enable manual control for GPU fan (pwm2)
echo 1 | sudo tee /sys/class/hwmon/hwmonX/pwm2_enable

# Return to automatic control
echo 2 | sudo tee /sys/class/hwmon/hwmonX/pwm1_enable
```

#### Set Fan Duty (pwm_input)

Duty cycle values range from 0-255:
- `0` = Fan off (minimum)
- `255` = Full speed (maximum)

```bash
# Set CPU fan to 50% duty (~128/255)
echo 128 | sudo tee /sys/class/hwmon/hwmonX/pwm1_input

# Set GPU fan to 75% duty (~192/255)
echo 192 | sudo tee /sys/class/hwmon/hwmonX/pwm2_input
```

> **Note:** PWM writes are rejected unless the fan is in manual mode (pwm_enable=1).

### Reading Current State

```bash
# Read current PWM duty value
cat /sys/class/hwmon/hwmonX/pwm1_input
cat /sys/class/hwmon/hwmonX/pwm2_input

# Read current mode (1=manual, 2=auto)
cat /sys/class/hwmon/hwmonX/pwm1_enable
cat /sys/class/hwmon/hwmonX/pwm2_enable
```

## Safety Considerations

### No Auto-Revert Timer
The driver does **not** implement a kernel-side auto-revert timer. When you enable manual fan control:
- You are responsible for returning fans to auto mode (`pwm_enable=2`) when done
- The kernel does not automatically restore automatic control
- Failure to revert may result in fans running at fixed speeds

### Suspend/Shutdown Behavior
The driver automatically reverts both fans to automatic mode during:
- System suspend
- System shutdown
- Driver unload

### Error Handling
If the EC becomes unresponsive:
- The driver invalidates its cached probe state
- Subsequent operations trigger a fresh re-probe
- Timeout after 500ms worst-case (see `ASUS_EC_TIMEOUT_US`)

## Troubleshooting

### Driver Won't Load

Check DMI match:
```bash
# Verify your system matches supported hardware
sudo dmidecode -s system-product-name
sudo dmidecode -s baseboard-product-name
```

Check kernel logs:
```bash
dmesg | grep asus_ec
```

Common messages:
- `"no matching DMI board, not loading"` - Your hardware is not in the supported list
- `"test-mode sanity check failed, not binding"` - Hardware passed DMI match but doesn't support the protocol

### I/O Port Conflicts

```bash
# Check if ports are in use
cat /proc/ioports | grep 25c
```

Error message: `"could not reserve I/O ports 0x25c-0x25d (in use by another driver?)"`

### Fan Control Not Working

1. Ensure fan is in manual mode first:
   ```bash
   cat /sys/class/hwmon/hwmonX/pwm1_enable  # Should return 1
   ```

2. Check dmesg for errors:
   ```bash
   dmesg | tail -50
   ```

## Technical Details

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Userspace                            │
│              (echo/cat to sysfs entries)                │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                    hwmon subsystem                      │
│         (pwm1_input, pwm1_enable, pwm2_*)               │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                   asus-ec driver                        │
│  ┌───────────────────────────────────────────────────┐  │
│  │              Per-device Mutex                     │  │
│  │        (serializes all EC transactions)           │  │
│  └───────────────────────────────────────────────────┘  │
│                           │                             │
│  ┌───────────────────────────────────────────────────┐  │
│  │           HealthyTable Protocol Layer             │  │
│  │    (select_fan, set_test_mode, set_duty)          │  │
│  └───────────────────────────────────────────────────┘  │
│                           │                             │
│  ┌───────────────────────────────────────────────────┐  │
│  │            I/O Port Transport Layer               │  │
│  │         (inb/outb to ports 0x25c/0x25d)           │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│              Embedded Controller (EC)                   │
│         (HealthyTable I/O-port protocol)                │
└─────────────────────────────────────────────────────────┘
```

### Key Design Decisions

1. **DMI Whitelist Only**: Driver only loads on explicitly verified hardware
2. **No ACPI Dependency**: Uses direct I/O port access for lower latency
3. **Userspace Responsibility**: No kernel-side watchdog for manual mode
4. **Stateful Protocol**: EC maintains "current fan" register between operations
5. **Cached Probe Result**: Interface version cached after first successful probe

## Credits

- **Author**: Niko Iwamura
- **Protocol Discovery**: PisonJay (HealthyTable/AsusFanControl userspace PoC)
- **License**: GPL-2.0-or-later

## References

- [Kernel driver submission thread](https://lore.kernel.org/lkml/) (search for asus-ec)
- [HealthyTable userspace PoC](https://github.com/search?q=healthhtable+asus) (reverse engineering source)

## Contributing

To add support for additional boards:
1. Verify the HealthyTable protocol works on your hardware
2. Add a new entry to `asus_ec_dmi_table[]` with exact DMI strings
3. Test thoroughly with the sanity checks enabled
4. Submit patches to the Linux kernel mailing list

> **Warning:** Always test on hardware you can afford to lose. Incorrect I/O port access can brick your system.