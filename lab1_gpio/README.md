# Lab 1 - GPIO Driver

In this lab you will:

1. Install the IDE (Code Composer Studio).
2. Write a GPIO driver from scratch using the reference material - TI
   DriverLib is not allowed.
3. Demonstrate control over an RGB LED based on user input.

This document is the complete assignment: what to build, how it's
graded, and how to get started.

## Overview

You will build a minimal GPIO driver for the MSPM0G3507 directly against its
registers. Do not use the TI DriverLib or no SysConfig-generated pin init. Use
the driver to read two push-buttons and drive an RGB LED. This is the
foundational bare-metal skill for the rest of the course: given a
datasheet and a Technical Reference Manual (TRM), you should be able to
find the memory-mapped registers that control a peripheral and drive them
correctly yourself, without depending on a vendor abstraction layer.

## Learning Objectives

- Import and build a Code Composer Studio (Theia) "nortos" project
  targeting the LP-MSPM0G3507 LaunchPad, and flash it over the on-board
  XDS110 debug probe.
- Read the MSPM0G350x TRM well enough to locate the GPIO and IOMUX
  register maps for a specific pin.
- Implement a concrete class against an instructor-provided abstract
  interface (pure virtual functions), using direct memory-mapped
  register access.
- Apply this repo's register-access conventions (`style.md` §S.10,
  Memory-Mapped I/O) to real hardware.

## Hardware

Board: [LP-MSPM0G3507 LaunchPad](https://www.ti.com/tool/LP-MSPM0G3507).

The on-board RGB LED (LED2) is wired to GPIO port B:

| Color | Pin  |
| ----- | ---- |
| Red   | PB26 |
| Green | PB27 |
| Blue  | PB22 |

The two push-buttons (S1, S2) are also wired to GPIO pins, but you'll need
to find exactly which ones yourself from the
[LP-MSPM0G3507 LaunchPad (Dev Board) User's Guide](https://www.ti.com/lit/slau873)
(TI literature number SLAU873).

## Grading Rubric (50 pt)

1. **Toolchain setup - 10 pts.** Install CCS along with the **MSPM0-SDK**
   (2.10.00.04) and **SysConfig** products - these are what let CCS
   resolve this project's `${COM_TI_MSPM0_SDK_*}` build variables. Then
   import the `cmpe146-2026-MSPM0G3507` repo as a new workspace folder,
   build this project, and flash/debug it as-is. Minimum required work
   for points: the application prints `"Hello, World"` - the starter
   code already does this via `SYSCFG_DL_init()` and `std::printf`.
1. **Implement `output_pin` driver for MSPM0G3507 - 10 pts** - Write a concrete
   class that implements the `output_pin` interface.
1. **Implement `input_pin` driver for MSPM0G3507 - 10 pts** - Write a concrete
   class that implements the `input_pin` interface.
1. **Switch Sense & LED Control demonstration - 10 pts.** Using your gpio
   drivers:
   - Pressing S1 turns the RGB LED to a color (any color).
   - Pressing S2 turns the RGB LED to a different color than S1.
   - Pressing both switches turns the RGB to a 3rd color.
   - The instructor will request that one of the colors be changed
     during the demo and you are responsible for demonstrate that you can
     changing the color, rebuild, and reflash, and demonstrate that the new
     color is working.
1. **Code review - 10 pts.** Submit a PR to your git repo targeting your
   `main` branch.

## Reference material

- **[MSPM0G350x Technical Reference Manual](https://www.ti.com/lit/ug/slau846/slau846.pdf)** -
  General Purpose Input/Output (GPIO) chapter and Input/Output Multiplexer
  (IOMUX) chapter. This is where the register offsets and bit fields you need
  actually live.
- **[MSPM0G3507 datasheet](https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf)** -
  IOMUX pin-to-peripheral function table, electrical
  characteristics, peripheral memory map.
- **[LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873)** -
  on-board LED/button wiring and BoosterPack pinout.
- **`style.md` §S.10 (Memory-Mapped I/O)** - the pattern this codebase
  uses for declaring and accessing hardware registers safely
  (`volatile` register-map structs, `reinterpret_cast` from a named
  `constexpr` base address, scoped to an anonymous namespace).

## Project layout

```text
lab1_gpio/
├── .project, .cproject, .ccsproject, .settings/   CCS project metadata - generated/maintained by CCS, don't hand-edit
├── targetConfigs/                                 Debug probe + device configuration (XDS110, MSPM0G3507)
├── lab1_gpio.syscfg                                SysConfig: clock tree + SWD debug pins only (no GPIO - that's your job)
└── lab1_gpio.cpp                                   main() - write your input_pin/output_pin classes and the lab
                                                     behavior here, against the interface in ../hal/gpio.hpp

../hal/gpio.hpp                                     Pure virtual input_pin/output_pin interfaces - given, don't modify.
                                                     Shared across all labs, not copied per-lab.
```

## HINT: IOMUX

The IOMUX value for GPIO is `1`. Its only stated in one area in the datasheet
in section `6.4 Connections for Unused Pins` that the IOMUX value for the GPIO
is value `1`. The section with the IOMUX information only has values 2 and
higher. Sometimes this happens with datasheets where information is either
missing or in a strange place thats hard to find.

# Lab 1 Explained

# 1. GPIO Register Access

## Before

The starter code did not directly provide the MSPM0G3507 register
addresses we needed.

## What was added

Created `get_reg()`:

```cpp
std::uint32_t volatile* get_reg(std::ptrdiff_t p_offset)
{
    if (m_port == 0) {
        return reinterpret_cast<std::uint32_t volatile*>(
            0x400A'0000UL + p_offset);
    }

    return reinterpret_cast<std::uint32_t volatile*>(
        0x400A'2000UL + p_offset);
} 
```

## Why?

The MSPM0G3507 has different base addresses for GPIOA and GPIOB:

GPIOA = 0x400A0000
GPIOB = 0x400A2000

The numbers such as 0x800 and 0x12D0 are NOT base addresses.
They are register offsets.

GPIO base address + 0x800 
so we get 0x400A0000 + 0x1290 by writing 

```cpp
get_reg(0x800)
get_reg(0x1290)
```

# 2. GPIO Register Offsets

## Given GIPO registers

0x0800 → PWREN
0x1280 → DOUT31_0
0x1290 → DOUTSET31_0
0x12A0 → DOUTCLR31_0
0x12D0 → DOESET31_0
0x1380 → DIN31_0

## Peripheral

```cpp
*get_reg(0x12D0) = pin_mask();

// uses the GPIO output-enable register.

*get_reg(0x1290) = pin_mask();

// sets the selected GPIO output HIGH.

*get_reg(0x12A0) = pin_mask();

// sets the selected GPIO output LOW.

*get_reg(0x1380)

// reads the GPIO input state.
```

## Distinction

BASE ADDRESS
    ↓
where the GPIO peripheral starts

OFFSET
    ↓
which register inside the GPIO peripheral

MASK
    ↓
which individual pin/bit inside that register

# 3. Pin Mask

## Definition

A GPIO register contains multiple bits, with each bit representing
one GPIO pin. 

For example, if:

m_pin = 18;

then:

1u << 18

creates a value with only bit 18 set.

Conceptually:

0000 0000 0000 0100 0000 0000 0000 0000
                  ↑
                bit 18

This lets the driver target one specific pin without targeting all
the other GPIO pins.

## Example

```cpp
*get_reg(0x1290) = pin_mask();
```

## What was used 

we had:

```cpp
std::uint32_t pin_mask() const
{
    return (1u << m_pin);
}
```

# 4. IOMUX Register

## Purpose

The purpose of this function is to find the correct IOMUX register
for a given (port, pin) combination.

```cpp
std::uint32_t volatile* get_iomux_reg()
```

## Lab Mapping

PA18 → PINCM 40

PB22 → PINCM 50
PB26 → PINCM 57
PB27 → PINCM 58

The driver uses a switch to translate the GPIO pin number
into its PINCM number.

Example:

case 26: pin_cm = 57; break;

means:

PB26
 ↓
PINCM 57

# 5. Finding the IOMUX Register Address

After finding the PINCM number, the driver calculates the address:

```cpp
iomux_base + (pin_cm * 4) //each register occupies 4 bytes
```

overall process:

GPIO pin
   ↓
PINCM number
   ↓
IOMUX base + PINCM × 4
   ↓
correct IOMUX register

# 6. get_iomux_reg()

The GPIO peripheral and the IOMUX are two different things.

The GPIO register tells the MCU things such as:

HIGH
LOW
READ INPUT
ENABLE OUTPUT

The IOMUX tells the MCU how the physical pin is connected/configured.

So for a pin such as PB26:

PB26
 ↓
IOMUX configuration
 ↓
connect pin to GPIO
 ↓
GPIOB bit 26
 ↓
control the LED

# 7. Driver Configuration

driver_configure() is implemented to actually write the IOMUX
configuration.

## Lab Use

The function starts with:

```cpp
std::uint32_t value = 0x40081;
```

Then the resistor configuration modifies the value:

```cpp
if (p_settings.resistor == lab1::pin_resistor::pull_up) {
    value |= 0x20000;
}
else if (p_settings.resistor == lab1::pin_resistor::pull_down) {
    value |= 0x10000;
}
```
So the configuration value is built by combining individual bits.

## Concepts

base configuration
+
pull-down bit
=
pull-down configuration

Then ↓

*get_iomux_reg() = value;

writes the configuration into the correct IOMUX register.

# 8. Output Level

```cpp
void driver_level(bool p_high)
{
    if (p_high) {
        *get_reg(0x1290) = pin_mask();
    }
    else {
        *get_reg(0x12A0) = pin_mask();
    }
}
```

Translates the simple command:

```cpp
red.level(true);
```

## Flow

For HIGHT:
red.level(true)
      ↓
driver_level(true)
      ↓
DOUTSET register
      ↓
pin mask
      ↓
GPIO pin becomes HIGH

For LOW:
red.level(false)
      ↓
driver_level(false)
      ↓
DOUTCLR register
      ↓
pin mask
      ↓
GPIO pin becomes LOW

# 9. Reading an Input (button)

## Masking Uses #2
```cpp
return (*get_reg(0x1380) & pin_mask()) != 0;
```
This is another use of masking.

0x1380 is the DIN register.

The DIN register contains the states of many GPIO pins.

The mask removes everything except the pin we care about.

## Example

DIN register
     ↓
& (1 << 18)
     ↓
only PA18 remains

Then:

!= 0

turns the result into a simple:

true  → pin is HIGH
false → pin is LOW

# 10. GPIO Classes

## class mspm0g_outpin

This class handles:

Configure output
Set HIGH
Set LOW
Read output

## class mspm0g_inpin

This class handles:

Configure input
Read input

# 11. What we did different

The destructor was removed because GPIO object does not need special clean up. All hardware registers are allocated and released. We got rid of it for purpose-targeting cleanliness.

```cpp
~mspm0g_outpin()
```

# 12. Main()

```cpp
mspm0g_outpin red(1, 26);
mspm0g_outpin green(1, 27);
mspm0g_outpin blue(1, 22);

mspm0g_inpin s1(0, 18);
mspm0g_inpin s2(1, 21);
```

## What these means

red   → Port B, Pin 26
green → Port B, Pin 27
blue  → Port B, Pin 22

S1 → Port A, Pin 18
S2 → Port B, Pin 21

## Conditional Statements

Finally, we used if/else for stages in the actual behavior.

If S1:
    RED

If S2:
    GREEN

If both:
    BLUE

If neither:
    OFF


#### Last but not least, here's the overall lesson for this lab: The board documentation tells me the physical pin. The IOMUX information tells me the PINCM mapping. The GPIO register information tells me the register offsets. The driver combines these pieces so that a simple command such as red.level(true) can ultimately control the physical hardware.