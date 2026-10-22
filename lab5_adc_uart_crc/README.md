# Lab 5 - ADC + UART + CRC (FreeRTOS)

For this lab, you will:

1. Implement an ADC driver with DriverLib that uses DMA to sample the
   on-chip temperature sensor.
2. Implement a register-level UART driver with an interrupt-driven circular
   receive buffer.
3. Implement a register-level driver for the on-chip CRC-32 accelerator and
   benchmark it against a supplied software CRC-32.
4. Convert raw ADC samples into temperatures.
5. Use FreeRTOS to build a request/response link between two boards: a
   **sensor** board and a **display** board.

## Use cases

- A central controller polling remote sensor nodes on demand
- Monitoring systems where the display is separate from the sensing hardware
- Any serial link where corrupted data must be caught before it's used

## Overview

Both roles live in `lab5_adc_uart_crc.cpp`. Set `this_device` near the top
of the file to pick the role, then rebuild and flash. The `if constexpr` in
`main()` compiles in only that role's code.

The exchange:

1. **Display:** wait for S1 or S2, then send `sample_request_command` (one
   byte) to the sensor board.
2. **Sensor:** take 64 temperature samples in one DMA transfer, convert
   them, append a CRC-32, and send the packet back. If the sensor board's S1
   or S2 (your choice) is held while sending, flip one bit in the packet
   after computing the CRC.
3. **Display:** check the CRC. If it matches, set the RGB LED from the
   readings: blue for cold, yellow for warm, red for hot (you pick the
   bands). If it doesn't, drop the packet, print a message, and turn the LED
   magenta.

**Packet format:** 64 temperature samples followed by a 4-byte CRC. The
sample encoding is your choice, as long as it resolves at least 0.5 degC (a
whole-degree integer can't). Agree on the encoding and byte order as a team
on day one, since both boards depend on it.

## Rules

- **DriverLib is required for the ADC driver**, including its DMA setup.
  The UART and CRC drivers are register-level.
- **All DriverLib calls go inside your driver classes.** Don't configure
  peripherals in SysConfig.

## FreeRTOS tasks

**Sensor board: three tasks**

- `request_task`: watches the UART for `sample_request_command`, then gives
  the `sample_request` semaphore.
- `sampler_task`: waits on `sample_request`, reads 64 ADC samples, converts
  them, and sends them to `batch_queue`.
- `transmit_task`: waits on `batch_queue`, computes both CRCs, and sends the
  packet.

**Display board: one task**

- `display_task`: sends the request on S1/S2, waits for the packet,
  validates the CRC, and sets the LED.

**How the sensor tasks pass data**

```text
UART RX
   |
request_task
   |   sample_request (binary semaphore): "take a batch now"
   v
sampler_task   <-- ADC0_IRQHandler (task notification): "buffer is full"
   |   batch_queue (queue): a copy of the 64 converted samples
   v
transmit_task
   |
UART TX
```

- `sample_request` carries no data, only the signal to start.
- `batch_queue` carries the data. Each item is a full copy of the 64
  samples, so the two tasks never share a buffer.
- The task notification lives inside `adc.read()`.

## Team roles

Each person owns one role and defends it at check-off. List the owners in
your PR description. Open your role's guide below; you only need to read
your own.

<details>
<summary><b>1. ADC: ADC/DMA driver, temperature conversion, <code>sampler_task</code></b></summary>

Two MSPM0 SDK examples cover most of this:
`examples/nortos/LP_MSPM0G3507/driverlib/adc12_14bit_resolution` (ADC with
DMA) and `adc12_internal_temp_sensor_mathacl` (temperature sensor). Their
`ti_msp_dl_config.c` files show the DriverLib calls SysConfig generates.
Move those calls into your driver.

**Constructor**

1. `DL_ADC12_reset()` and `DL_ADC12_enablePower()` on `ADC0`, then wait for
   it to power up.
2. `DL_ADC12_setClockConfig()`: SYSOSC as the clock source.
3. `DL_ADC12_initSingleSample()`: repeat mode, software trigger, 12-bit,
   unsigned.
4. `DL_ADC12_configConversionMem()`: MEM0 reads `DL_ADC12_INPUT_CHAN_11`
   (the temperature sensor) against the VDDA reference.
5. `DL_ADC12_setSampleTime0()`: at least 12.5 us. Compute the cycle count
   from your ADC clock.
6. `DL_ADC12_setDMASamplesCnt(ADC0, 1)` and
   `DL_ADC12_enableDMATrigger(ADC0, DL_ADC12_DMA_MEM0_RESULT_LOADED)`. Enable
   the `DL_ADC12_INTERRUPT_DMA_DONE` interrupt and `ADC0_INT_IRQn` in the
   NVIC.
7. `DL_DMA_initChannel()`: trigger `DMA_ADC0_EVT_GEN_BD_TRIG`, half-word
   source and destination, fixed source address, incrementing destination.
   The source address is
   `DL_ADC12_getMemResultAddress(ADC0, DL_ADC12_MEM_IDX_0)`.
8. `DL_ADC12_enableConversions()`.

**Each `read()`**

1. Disable the DMA channel, point its destination at the buffer, set the
   transfer size to the buffer's size, and re-enable it.
2. `DL_ADC12_enableDMA()`. The ADC clears this bit after every completed
   transfer.
3. Save the calling task's handle from `xTaskGetCurrentTaskHandle()`
   somewhere the interrupt handler can reach, like a file-scope variable.
   Do this before starting the conversion.
4. `DL_ADC12_startConversion()`, then sleep with
   `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)`.
5. Once woken, `DL_ADC12_stopConversion()`, then upscale each 12-bit sample
   to 16 bits as `adc.hpp` requires.

**Interrupt handler**

1. Define `extern "C" void ADC0_IRQHandler()`. DMA completion is reported
   through the ADC's interrupt.
2. Check that `DL_ADC12_getPendingInterrupt(ADC0)` returns
   `DL_ADC12_IIDX_DMA_DONE`.
3. Declare `BaseType_t higher_priority_task_woken = pdFALSE;` and call
   `vTaskNotifyGiveFromISR(saved_handle, &higher_priority_task_woken)`.
4. End with `portYIELD_FROM_ISR(higher_priority_task_woken)` so the woken
   task runs as soon as the handler returns.

**Temperature conversion**

- `TEMP_SENSE0` at `0x41C4003C` holds the factory calibration value
  (`DL_SYSCTL_getTempCalibrationConstant()` reads it). It's the 12-bit ADC
  code measured at 30 degC with the 3.3 V VDD reference.
- The sensor's output changes by TSc = -1.8 mV/degC (datasheet section
  7.14).
- `T = 30 + (V_sample - V_trim) / TSc`, where each voltage is
  `3.3 V * code / 4096`.
- Your samples are upscaled to 16 bits, so scale either them or the
  calibration value so both use the same units.

**`sampler_task`**

1. Block in `xSemaphoreTake(sample_request, portMAX_DELAY)`.
2. Call `adc.read()` on a `std::array<std::uint16_t, 64>`.
3. Convert each sample to `temperature_sample_t`.
4. `xQueueSend(batch_queue, &samples, portMAX_DELAY)` copies the converted
   array into the queue.

</details>

<details>
<summary><b>2. UART: UART driver</b></summary>

- Register-level. `configure()` computes the baud rate dividers from the
  requested settings.
- Get polled transmit and receive working first so your team can test
  between boards early. Then move receive to the interrupt.
- On receive, the RX interrupt copies each byte into a circular buffer, and
  `receive_cursor()` returns where the next byte will go. See its docs in
  `serial.hpp`.
- Make the buffer larger than one packet. A full lap around the buffer looks
  like zero new bytes. 256 bytes is a safe size.
- The handler is `extern "C" void UARTn_IRQHandler()` for your instance
  (`UART0` through `UART3`).
- Test alone by wiring TX to RX on one board.

</details>

<details>
<summary><b>3. CRC: CRC driver, benchmark, <code>transmit_task</code></b></summary>

**Driver**

- Register-level, against the on-chip CRC accelerator.
- Output must match `crc32_software` (standard CRC-32, as used by zlib).
  Check the accelerator's bit-order options and whether it applies the final
  XOR.

**`transmit_task`**

1. Block in `xQueueReceive(batch_queue, &samples, portMAX_DELAY)`.
2. Compute the CRC over the samples with `crc32_hardware` and
   `crc32_software`, timing each with `lab2::steady_clock`. Print both
   durations.
3. Read the corruption button. If it's held, flip one bit in the samples.
4. Write the samples, then the CRC.

</details>

<details>
<summary><b>4. Application: <code>request_task</code>, <code>display_task</code>, semaphore, queue, <code>main()</code></b></summary>

**Setup in `main()`**

- Include `semphr.h` and `queue.h`.
- Before `vTaskStartScheduler()`, create `sample_request` with
  `xSemaphoreCreateBinaryStatic()`, `batch_queue` with
  `xQueueCreateStatic()` (a `StaticQueue_t` plus storage for one
  `std::array<temperature_sample_t, 64>`), and the three sensor tasks with
  `xTaskCreateStatic()`, each with its own stack and TCB.
- The serial driver is shared: `request_task` only reads from it, and
  `transmit_task` only writes to it.

**`request_task`**

- Every few ms (`vTaskDelay()`), compare `receive_cursor()` with the last
  position you read. When `sample_request_command` arrives, call
  `xSemaphoreGive(sample_request)`.

**`display_task`**

1. On S1 or S2, save the current `receive_cursor()`, then send
   `sample_request_command`.
2. Wait until the full packet (64 samples plus the 4-byte CRC) has arrived
   after the saved position. If it takes more than 500 ms, print a timeout
   message.
3. Recompute the CRC over the samples and compare it to the one received.
   On a mismatch, print a message and set the LED magenta. Otherwise,
   average the samples and set the LED color.

</details>

Teams of 5 split role 4: one person takes `request_task`, the semaphore, and
the queue, and the other takes `display_task`.

Until a teammate's part works, use a stand-in behind the same interface: an
`adc16_stream` that returns a ramp (each sample one higher than the last),
`crc32_software` for the CRC, or a `serial` that prints whatever it's asked
to write.

**Goal for the end of week 1:** `crc32_hardware` matches `crc32_software`,
and the two boards can exchange bytes over UART.

## Hardware

Two [LP-MSPM0G3507 LaunchPads](https://www.ti.com/tool/LP-MSPM0G3507), one
per role. Pick any UART instance, cross-connect TX/RX between the boards,
and connect their grounds. See [lab1](../lab1_gpio/README.md#hardware) for
the RGB LED and S1/S2 pins. The temperature sensor is inside the chip and
needs no wiring.

## Grading Rubric (120 pt)

The whole team shares one grade. Roles decide who answers which questions at
check-off, not who earns which points.

### Role 1: ADC (25 pts)

- **ADC + DMA - 15 pts.** `adc_driver` fills the whole buffer in one DMA
  transfer and sleeps the calling task until the DMA-done interrupt wakes it.
- **Temperature conversion - 10 pts.** Samples convert to temperatures with
  at least 0.5 degC resolution.

### Role 2: UART (25 pts)

- **UART transmit - 10 pts.** Transmits at the configured baud rate.
- **UART receive - 15 pts.** Receives into an interrupt-driven circular
  buffer, and `receive_cursor()` behaves as `serial.hpp` describes.

### Role 3: CRC (20 pts)

- **CRC driver - 15 pts.** `crc32_hardware` matches `crc32_software` and is
  measurably faster.
- **Corrupted data demonstration - 5 pts.** Holding the sensor board's
  button flips a bit, and the display board catches it.

### Role 4: Application (25 pts)

- **Display role - 15 pts.** S1/S2 requests a batch and the LED shows the
  temperature. Bad CRCs turn the LED magenta, and both bad CRCs and timeouts
  print a message.
- **Sensor task architecture - 10 pts.** Three statically allocated tasks,
  the `sample_request` semaphore, and `batch_queue`, as described above.

### Whole team (25 pts)

- **End-to-end exchange demo - 15 pts.** Everything working together:
  pressing S1/S2 on the display board gets a fresh batch from the sensor
  board every time, across repeated presses, without resetting either board.
- **Code submission - 10 pts.** A PR to your repo's `main` branch, with role
  owners listed in the description.

Teams of 5 split role 4's points along with its work: display role to one
person, task architecture to the other.

## Reference material

- [MSPM0G3507 datasheet](https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf):
  UART pin table and temperature sensor specs (section 7.14).
- [MSPM0G350x Technical Reference Manual](https://www.ti.com/lit/ug/slau846e/slau846e.pdf):
  ADC, DMA, UART, and CRC chapters.
- MSPM0 SDK examples: `adc12_14bit_resolution` and
  `adc12_internal_temp_sensor_mathacl`.

<!-- cspell:ignore mathacl SYSOSC VDDA IIDX semphr -->
