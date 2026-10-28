# CMPE 146 Final Project Spec

Oct 27, 2026 · @Khalil

## Overview

Build an interactive embedded system of your team's choosing on the MSPM0G3507 or MSP432 LaunchPad. Requirements are deliberately light; what matters is that the project works reliably and every member owns a part of it.

- **Team size:** 4–5 (your Lab 5 team)
- **Duration:** 5 weeks
- **Weight:** 45% of the course grade
- **P****re****s****entatio****n**** ****D****a****y****:** before finals week Dec 2nd (Wednesday) & 3rd (Thursday)
- **Presentation ****d****urati****on****:**&#32;
  - 5 to 10 minutes for slides
  - 5 minutes to live demo the project

## Requirements

1. **Concurrency.** At least two FreeRTOS tasks doing meaningful work.
   - Meaningful means each task owns a peripheral or a stage of the data flow (e.g. sensing, control, UI).
   - Tasks communicate through a FreeRTOS primitive: queue, semaphore, task notification, or event group.
   - A system "heartbeat" indicator (LED blink or similar) is required so we can see the system is alive. It does not count toward the two tasks, and neither does a watchdog.
2. **Input and output.** At least one input and one output device that are not on the LaunchPad.
   - The onboard RGB LED and buttons do not count.
   - The input must visibly affect the output in real time.
   - Examples of inputs: capacitive touch, IMU, distance sensor, joystick, keypad, rotary encoder, environmental sensor, external temperature sensor (the LaunchPad's onboard temperature sensor doesn't count).&#32;
   - Examples of outputs: servos, motors, displays, LED matrices, audio.
   - A host PC app can count as the output only if it is interactive (live view, controls), not a log dump.
3. **Portable drivers.** At least two device drivers written against the interfaces from class.
   - No MCU register access inside any driver itself.
   - At least one must be for an external device your project uses.
4. **Ownership.** Teams divide the work among themselves; in the presentation, each member presents the part they did.
5. **Polish.**
   - Boots and runs standalone on USB or battery power, with no debugger attached and no reflashing between demos.
   - Survives a 5-minute public demo without a reset.
   - **Recommended:** record at least one video of the project working as a backup for reduced points.&#32;
6. **Presentation:** 5 to 10 min slide show presentation with 5 min for a demo. It must include these (6 elements)
   1. A description of what the project is, why you decided to do it, and what each team member did
   2. A hardware wire diagram of the project with high level bill of materials
   3. Diagram of the software architecture
      1. Each task and does (name of task in presentation should match code)
      2. How tasks communicate with each other.
      3. Drivers used and made to accomplish the task's objectives
      4. At least one logic analyzer capture of inter-device communication
   4. Any issues you encountered when developing the project (optional if none really occurred)

## Hardware Guidelines

- Prefer 3.3V-logic modules. Check before buying: some common parts (e.g. the HC-SR04 ultrasonic sensor) are 5V.
- Servos and motors get their own power supply, never the LaunchPad's rails. Ground between power supplies must be shared.

**Power options**

- **For MCU and low power devices like sensors:** USB from a laptop or a USB power bank.
- **Servos and small motors:** a 4×AA or 4×NiMH pack (about 5–6V), or a USB power bank through a USB breakout.
- **Higher-voltage motors (e.g. 9–12V)**: a USB-C PD trigger board on a PD-capable power bank or charger, followed by a regulator for anything that needs 5V or 3.3V. Never connect the trigger output straight to the LaunchPad or servos.
- Lab bench supplies are fine during development.

**Wireless**

- **Good choices:** nRF24L01 radios (SPI), or BLE UART-passthrough modules (e.g. HM-10) when a phone or PC is on the other end.
- **D****i****f****f****i****cu****l****t****:** Wi-Fi modules driven by AT commands (e.g. ESP-01). Expect to write an asynchronous reply parser, run the module as its own access point or on a phone hotspot (campus Wi-Fi will not work), and give it a dedicated 3.3V regulator for its \~300 mA current spikes.

## Milestones and grading

| Week | Milestone | What to submit |
| --- | --- | --- |
| 1 | Proposal | One paragraph describing the project and a parts list with links. Must be approved before ordering parts. |
| 6 | Final | Final presentation to the whole lab and public demo, before finals week. |

Grading is split by when each item is observed: 60 points at the presentation and 40 points from the code afterwards. The assignment is worth 45% of your grade.

**At the presentation (60)**

| Item | Full credit | Partial credit | Zero |
| --- | --- | --- | --- |
| Demo works (20) | Works live as described | (10) works after a reset or workaround, or shown only by backup video | Does not work |
| Input and output (10) | External input visibly drives external output in real time | (5) both present but not interactive, or only one is external | Onboard devices only |
| Standalone boot (5) | Runs from power-on with no debugger | — | Needs the debugger or a reflash |
| Project overview slide (5) | Clearly explains what the project is, why you chose it, and what each member did | (2) present but unclear or incomplete | Missing |
| Wiring diagram slide (4) | Clear wiring diagram with a high-level bill of materials | (2) present but unclear or incomplete | Missing |
| Tasks slide (4) | Clearly explains each task and what it does | (2) present but unclear or incomplete | Missing |
| Task communication slide (4) | Clearly explains how the tasks communicate with each other | (2) present but unclear or incomplete | Missing |
| Drivers slide (4) | Clearly explains the drivers used and written | (2) present but unclear or incomplete | Missing |
| Logic analyzer capture (4) | At least one capture of inter-device communication, explained | (2) present but unclear or incomplete | Missing |

Issues encountered during development are optional in the presentation and not graded.

**In the code afterwards (40)**

Graded after the presentation, using the slides' task diagram as the map. A mismatch between the diagram and the code drops that item to partial credit.

| Item | Full credit | Partial credit | Zero |
| --- | --- | --- | --- |
| Concurrency (15) | Two or more meaningful tasks communicating through a FreeRTOS primitive, plus heartbeat, matching the diagram | (7) one task is trivial, data is shared through unprotected globals, or the code doesn't match the diagram | Single task or superloop |
| Portable drivers (15) | Two drivers with no register access, at least one for an external device, matching the diagram | (7) only one portable driver, or register access leaking in, or the code doesn't match the diagram | No portable drivers |
| Code organization (10) | Drivers, tasks and application in separate modules with clear names | (5) works but tangled (unnecessary code, code or constants in the wrong place, two-phase initialization for driver objects) | — |

## Example projects

You are not limited to these. A good project can be explained in one sentence and shown in a 30-second video.

| Project | Input | Output | Notes |
| --- | --- | --- | --- |
| 2- or 3-axis action camera gimbal | IMU (I2C), joystick to set target | RC servos (PWM) | Sensor/filter task and servo task run at different rates. 3-axis needs a 9-axis IMU (e.g. ICM-20948) to stop yaw drift. Brushed motors instead of servos = stretch. |
| Handheld LED-matrix game | Joystick (ADC), buttons | MAX7219 8×8 matrix (SPI), buzzer | Snake, or a sequel to the Lab 2 rhythm game. |
| Tilt maze or digital level | IMU (I2C) | OLED or LED matrix |  |
| Touch synth or theremin | Capacitive touch or ToF distance sensor (I2C) | Audio via PWM or DAC + small amp |  |
| Keypad door lock | 4×4 keypad (GPIO scan) | Servo latch, OLED | Lockout after wrong codes. |
| Remote-controlled RC car | Handheld controller with joysticks or a tilt IMU | Motor controller (PWM) for drive, steering servo | Controller and car linked by radio (e.g. nRF24L01 over SPI). The car stops if the link drops. Drive motor needs its own power. The most work on this list; start early. |
| Sun-tracking solar panel | Light sensors (ADC), INA219 power monitor (I2C) | Two servos aiming a small panel, OLED | Reports how much more power it collects than a fixed panel. Demo indoors with a flashlight. |
| Pocket thermal camera | AMG8833 8×8 thermal sensor (I2C) | Small color display | Interpolated heat map drawn as rectangles. The sensor breakout is about $45. |
| Person-following desk fan | AMG8833 8×8 thermal sensor (I2C) | Pan-tilt RC servos (PWM), fan (PWM through a MOSFET) | Combines the thermal sensor driver, servo control, fan control, and tracking code that finds the hottest spot in view and steers the fan toward it. |
| Servo motor from scratch using DC Brushed motor | QEI encoder (timer or edge interrupt) OR magnetic encoder (I2C) and potentiometers (ADC) | H-bridge (DRV8871) | Consider buying a brushed DC motor with an encoder already on it to simplify the design. Then use the H-bridge to drive the motor. Have some means to control the position and velocity of the output shaft of the motor. |
| Digital sand hourglass | Accelerometer (I2C) | Two 8×8 LED matrices (MAX7219, SPI) | Physics engine display of grains of "sand" fall according to tilt; flipping it over restarts the timer. |
| Bike safety light | Handlebar buttons, IMU (I2C) | Rear LED matrix | Buttons show turn arrows; the IMU detects braking and flashes a brake light. |
| RFID lab tool checkout | RC522 RFID reader (SPI) | OLED, buzzer | Tap a card to check a tool in or out; the display shows who has what, and an unknown card sounds the buzzer. |
| CO2 ventilation monitor | SCD40 CO2 sensor (I2C), button or encoder | Color LED bar, OLED trend graph | Alerts when the room needs fresh air. |
| Smart PC fan controller | Temperature sensor, fan tach (timer capture), buttons | Fan PWM (25 kHz), OLED | Requires a 4-pin fan with speed output signal. Closed-loop RPM following a temperature curve, with a stall alarm. The fan needs 12V (see Power options). |
| Ball-and-beam balancer | Distance sensor (ToF, I2C), knob for target position | Servo tilting the beam | Harder: expect significant PID tuning time. |

**Stretch tier:** self-balancing or line-following robots. Only with instructor approval at the proposal.
