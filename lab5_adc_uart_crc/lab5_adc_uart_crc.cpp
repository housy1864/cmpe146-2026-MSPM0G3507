// cspell:ignore VDDA IIDX DDEVICE

#include <array>
#include <cstdint>
#include <cstdio>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <ti/driverlib/driverlib.h>

#include "../hal/adc.hpp"
#include "../hal/crc.hpp"
#include "../hal/gpio.hpp"
#include "../hal/serial.hpp"
#include "../hal/timer.hpp"

// =============================================================================
// Role 1: ADC
// =============================================================================

// TODO(lab, role 1): Make this class inherit from and implement the
// `lab5::adc16_stream` interface (`adc.hpp`) using DriverLib, wired to the
// internal temperature sensor (ADC0 channel 11). Follow the "1. ADC" guide in
// the README:
//   - Constructor: power up ADC0, configure MEM0 for channel 11 against VDDA,
//     set the sample time, and set up the ADC's DMA trigger and DMA channel.
//   - driver_read(): re-arm the DMA for the given buffer, save the calling
//     task's handle, start the conversion, then sleep with ulTaskNotifyTake()
//     until ADC0_IRQHandler() wakes you. Upscale each 12-bit sample to 16 bits
//     before returning.

// The FreeRTOS task currently waiting for an ADC DMA transfer.
static TaskHandle_t adc_waiting_task = nullptr;

// Adding a diagonstic flag
static volatile bool adc_read_success = false;

// Use DMA channel 0.
static constexpr uint8_t adc_dma_channel = 0;

class adc_driver : public lab5::adc16_stream
{
public:
    adc_driver()
    {
        // 1. Reset and power up ADC0.
        DL_ADC12_reset(ADC0);
        DL_ADC12_enablePower(ADC0);

        // Allow the ADC peripheral to power up.
        delay_cycles(16000);

        // 2. Configure ADC clock: SYSOSC, 24-32 MHz, divide by 1.
        static const DL_ADC12_ClockConfig clock_config = {
            .clockSel = DL_ADC12_CLOCK_SYSOSC,
            .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
            .divideRatio = DL_ADC12_CLOCK_DIVIDE_1,
        };

        DL_ADC12_setClockConfig(ADC0, &clock_config);

        // 3. Configure repeated, software-triggered, 12-bit conversions.
        DL_ADC12_initSingleSample(
            ADC0,
            DL_ADC12_REPEAT_MODE_ENABLED,
            DL_ADC12_SAMPLING_SOURCE_MANUAL,
            DL_ADC12_TRIG_SRC_SOFTWARE,
            DL_ADC12_SAMP_CONV_RES_12_BIT,
            DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED
        );

        // 4. Configure MEM0 for the internal temperature sensor.
        DL_ADC12_configConversionMem(
            ADC0,
            DL_ADC12_MEM_IDX_0,
            DL_ADC12_INPUT_CHAN_11,
            DL_ADC12_REFERENCE_VOLTAGE_VDDA,
            DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
            DL_ADC12_AVERAGING_MODE_DISABLED,
            DL_ADC12_BURN_OUT_SOURCE_DISABLED,
            DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
            DL_ADC12_WINDOWS_COMP_MODE_DISABLED
        );

        // 5. Set the ADC sample time.
        // At 32 MHz, 400 ADC clock cycles = 12.5 microseconds.
        DL_ADC12_setSampleTime0(ADC0, 400);

        // 6. Configure ADC DMA requests.
        // One ADC result is transferred for each DMA request.
        DL_ADC12_setDMASamplesCnt(ADC0, 1);

        DL_ADC12_enableDMATrigger(
            ADC0,
            DL_ADC12_DMA_MEM0_RESULT_LOADED
        );

        // Enable DMA channel 0 completion event.
        DL_DMA_clearEventsStatus(
            DMA,
            DL_DMA_EVENT_CHANNEL0
        );

        DL_DMA_enableEvent(
            DMA,
            DL_DMA_EVENT_CHANNEL0
        );

        // Enable the DMA interrupt in the NVIC.
        NVIC_EnableIRQ(DMA_INT_IRQn);

        // 7. Configure DMA channel 0.
        static const DL_DMA_Config dma_config = {
            .trigger = DMA_ADC0_EVT_GEN_BD_TRIG,
            .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
            .transferMode = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
            .extendedMode = DL_DMA_NORMAL_MODE,
            .srcWidth = DL_DMA_WIDTH_HALF_WORD,
            .destWidth = DL_DMA_WIDTH_HALF_WORD,
            .srcIncrement = DL_DMA_ADDR_UNCHANGED,
            .destIncrement = DL_DMA_ADDR_INCREMENT,
        };

        DL_DMA_initChannel(
            DMA,
            adc_dma_channel,
            &dma_config
        );

        // ADC result register is the fixed DMA source.
        DL_DMA_setSrcAddr(
            DMA,
            adc_dma_channel,
            DL_ADC12_getMemResultAddress(
                ADC0,
                DL_ADC12_MEM_IDX_0
            )
        );

        // 8. Enable ADC conversions.
        DL_ADC12_enableConversions(ADC0);
    }

private:
    // Expand a 12-bit ADC value to the 16-bit format required by adc.hpp.
    static std::uint16_t upscale_12_to_16(std::uint16_t value)
    {
        return static_cast<std::uint16_t>(
            (value << 4) | (value >> 8)
        );
    }

void driver_read(std::span<std::uint16_t> buffer) override
{
    adc_read_success = false;

    if (buffer.empty())
        return;

    // Disable DMA before reconfiguring the transfer.
    DL_DMA_disableChannel(DMA, adc_dma_channel);

    // Set destination buffer.
    DL_DMA_setDestAddr(
        DMA,
        adc_dma_channel,
        static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(buffer.data())
        )
    );

    // Transfer the requested number of 16-bit samples.
    DL_DMA_setTransferSize(
        DMA,
        adc_dma_channel,
        static_cast<uint16_t>(buffer.size())
    );

    // Clear stale notifications and DMA interrupt status.
    ulTaskNotifyTake(pdTRUE, 0);

    DL_DMA_clearInterruptStatus(
        DMA,
        DL_DMA_INTERRUPT_CHANNEL0
    );

    // Save the task that must be awakened.
    adc_waiting_task = xTaskGetCurrentTaskHandle();

    // Start DMA and ADC.
    DL_DMA_enableChannel(DMA, adc_dma_channel);
    DL_ADC12_enableDMA(ADC0);
    DL_ADC12_startConversion(ADC0);

    // Wait for DMA channel completion.
    uint32_t notification = ulTaskNotifyTake(
        pdTRUE,
        pdMS_TO_TICKS(1000)
    );

    // Stop the peripherals.
    DL_ADC12_stopConversion(ADC0);
    DL_ADC12_disableDMA(ADC0);
    DL_DMA_disableChannel(DMA, adc_dma_channel);

    adc_waiting_task = nullptr;

    if (notification == 0)
    {
        std::printf("ERROR: DMA channel 0 timeout!\n");
        return;
    }

    // Convert all 12-bit samples to 16-bit samples.
    for (auto& sample : buffer)
    {
        sample = upscale_12_to_16(sample);
    }

    adc_read_success = true;
}

};


// TODO(lab, role 1): Store the handle of the task waiting in
// adc_driver::driver_read() somewhere this handler can reach. When
// DL_ADC12_getPendingInterrupt(ADC0) returns DL_ADC12_IIDX_DMA_DONE, wake that
// task with vTaskNotifyGiveFromISR() and finish with portYIELD_FROM_ISR().

// ADC interrupt handler.
extern "C" void DMA_IRQHandler(void)
{
    if (DL_DMA_getPendingInterrupt(DMA) ==
        DL_DMA_EVENT_IIDX_DMACH0)
    {
        DL_DMA_clearInterruptStatus(
            DMA,
            DL_DMA_INTERRUPT_CHANNEL0
        );

        BaseType_t higher_priority_task_woken = pdFALSE;

        if (adc_waiting_task != nullptr)
        {
            vTaskNotifyGiveFromISR(
                adc_waiting_task,
                &higher_priority_task_woken
            );
        }

        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

// ================================
// Temperature Conversion Function
// ================================


// Convert an ADC reading into temperature in units of 0.1 degrees Celsius.
static std::int16_t convert_to_temperature(std::uint16_t sample)
{
    // The ADC result was expanded from 12 bits to 16 bits.
    // Recover the original 12-bit ADC code.
    const std::uint16_t sample_code = sample >> 4;

    // Factory calibration code measured at 30 degrees Celsius.
    const std::uint16_t trim =
        DL_SYSCTL_getTempCalibrationConstant();

    // Convert the ADC difference into degrees Celsius.
    constexpr float reference_voltage = 3.3f;
    constexpr float adc_max_code = 4096.0f;
    constexpr float sensor_slope = -0.0018f;

    const float sample_voltage =
        reference_voltage * sample_code / adc_max_code;

    const float trim_voltage =
        reference_voltage * trim / adc_max_code;

    const float temperature_c =
        30.0f + (sample_voltage - trim_voltage) / sensor_slope;

    // Store temperature in tenths of a degree Celsius.
    // Example: 25.3 C becomes 253.
    const float scaled_temperature = temperature_c * 10.0f;

    if (scaled_temperature > 32767.0f)
        return 32767;

    if (scaled_temperature < -32768.0f)
        return -32768;

    return static_cast<std::int16_t>(scaled_temperature + 0.5f);
}


// =============================================================================
// Role 2: UART
// =============================================================================

// TODO(lab, role 2): Make this class inherit from and implement the
// `lab6::serial` interface (`serial.hpp`), written register-level - no
// DriverLib. driver_configure() computes the baud rate dividers from the
// requested settings. Reception fills a circular buffer from the UART's RX
// interrupt; driver_receive_cursor() returns where the next byte will be
// written. Make the buffer larger than one packet (256 bytes is safe).
//
// Add the matching `extern "C" void UARTn_IRQHandler()` for the UART instance
// you pick (UART0 through UART3).
class serial_driver
{};

// =============================================================================
// Role 3: CRC
// =============================================================================

// TODO(lab, role 3): Make this class inherit from and implement the
// `lab6::crc32` interface (`crc.hpp`), written register-level against the
// on-chip CRC accelerator - no DriverLib. Its output must match
// crc32_software below for the same input. Check the accelerator's bit-order
// options and whether it applies the final XOR.
class crc32_hardware
{};

// Supplied reference implementation - do not modify. This is the standard
// reflected CRC-32 (IEEE 802.3 / zlib) algorithm, computed bit-by-bit in
// software. Your crc32_hardware driver's output must match this for the
// same input, and should be measurably faster.
class crc32_software : public lab6::crc32
{
private:
  void driver_reset() override
  {
    m_crc = initial_value;
  }

  void driver_update(std::span<std::uint8_t const> p_data) override
  {
    for (auto const byte : p_data) {
      m_crc ^= byte;
      for (int bit = 0; bit < 8; bit++) {
        std::uint32_t const mask = -(m_crc & 1U);
        m_crc = (m_crc >> 1) ^ (polynomial & mask);
      }
    }
  }

  std::uint32_t driver_checksum() override
  {
    return m_crc ^ initial_value;
  }

  static constexpr std::uint32_t polynomial = 0xEDB8'8320U;
  static constexpr std::uint32_t initial_value = 0xFFFF'FFFFU;

  std::uint32_t m_crc = initial_value;
};

namespace {
// =============================================================================
// Shared between both boards
// =============================================================================

// The one-byte message the display board sends to ask the sensor board for
// a fresh batch of readings.
constexpr std::uint8_t sample_request_command = 0xA5;

constexpr std::size_t samples_per_batch = 64;

// TODO(lab, team): Agree on an encoding for a single temperature sample that
// resolves at least 0.5 degC. A fixed-point scheme (e.g. int16_t in units of
// 0.1 degC) or a float both work. Both boards must use the same encoding and
// byte order.
using temperature_sample_t = std::int16_t;

// One batch of converted samples. This is the item type of batch_queue.
using temperature_batch = std::array<temperature_sample_t, samples_per_batch>;

// A packet on the wire: the batch's bytes, followed by the 4-byte CRC-32.
constexpr std::size_t packet_size =
  sizeof(temperature_batch) + sizeof(std::uint32_t);

// =============================================================================
// Sensor board: semaphore and queue (role 4 creates these in main())
// =============================================================================

// request_task -> sampler_task: "take a batch now". Carries no data.
// TODO(lab, role 4): Create with xSemaphoreCreateBinaryStatic().
SemaphoreHandle_t sample_request = nullptr;

// sampler_task -> transmit_task: a copy of one temperature_batch.
// TODO(lab, role 4): Create with xQueueCreateStatic(), depth 1, item size
// sizeof(temperature_batch).
QueueHandle_t batch_queue = nullptr;

// =============================================================================
// Sensor board: tasks
// =============================================================================

// TODO(lab, role 4): Every few ms (vTaskDelay()), compare the serial driver's
// receive_cursor() with the last position you read. When
// sample_request_command arrives, xSemaphoreGive(sample_request). This task
// only reads from the serial driver.
void request_task(void*)
{
  while (true) {
  }
}

// TODO(lab, role 1): Construct an adc_driver. Loop forever:
//   1. xSemaphoreTake(sample_request, portMAX_DELAY)
//   2. Fill a std::array<std::uint16_t, samples_per_batch> with one call to
//      adc_driver::read() (one DMA transfer, one task sleep).
//   3. Convert each raw sample into a temperature_batch. See "Temperature
//      conversion" in the README for the calibration constants.
//   4. xQueueSend(batch_queue, &batch, portMAX_DELAY)
void sampler_task(void*)
{
  // Create our ADC driver.
  adc_driver adc;

  // Buffer for 64 raw ADC measurements.
  std::array<std::uint16_t, samples_per_batch> raw_samples;

  // Buffer for 64 converted temperature measurements.
  temperature_batch temperatures;

  while (true)
  {
      // 1. Wait until the display board requests measurements.
      xSemaphoreTake(sample_request, portMAX_DELAY);

      // 2. Collect 64 ADC samples using DMA.
      // This call sleeps until the DMA interrupt finishes.
      adc.read(raw_samples);

      // 3. Convert all 64 ADC samples into temperatures.
      for (std::size_t i = 0; i < samples_per_batch; i++)
      {
          temperatures[i] =
              convert_to_temperature(raw_samples[i]);
      }

      // 4. Send the temperature batch to the queue.
      // The queue makes its own copy of the array.
      xQueueSend(
          batch_queue,
          &temperatures,
          portMAX_DELAY
      );
  }
}

// TODO(lab, role 3): Construct a crc32_hardware and a lab1::input_pin for
// your corruption-test button (S1 or S2). Loop forever:
//   1. xQueueReceive(batch_queue, &batch, portMAX_DELAY)
//   2. Compute the CRC-32 over the batch's bytes with both crc32_hardware and
//      crc32_software, timing each with a lab2::steady_clock. Print both
//      durations.
//   3. If the button is held, flip one bit in the batch (after the CRC, so
//      the CRC no longer matches).
//   4. Write the batch, then the CRC, with the serial driver. This task only
//      writes to the serial driver.
void transmit_task(void*)
{
  while (true) {
  }
}

// =============================================================================
// Display board: task
// =============================================================================

// TODO(lab, role 4): Construct a serial_driver, a crc32_hardware, two
// lab1::input_pin (S1 and S2), and three lab1::output_pin (the RGB LED).
// Loop forever:
//   1. Wait for S1 or S2 to be pressed.
//   2. Save the current receive_cursor(), then write sample_request_command.
//   3. Wait until packet_size bytes have arrived after the saved position. If
//      they don't arrive within 500 ms, print a timeout message and go back
//      to step 1.
//   4. Recompute the CRC over the received batch and compare it to the
//      received CRC. On a mismatch, print a message and set the LED magenta
//      (red + blue).
//   5. Otherwise, average the batch and set the LED: blue for cold, yellow
//      for warm, red for hot. You pick the bands.
void display_task(void*)
{
  while (true) {
  }
}

// =============================================================================
// Task storage
// =============================================================================

// Resize task_stack_words if necessary. Crashes can be caused by stack
// overflows.
constexpr std::uint32_t task_stack_words = 512;

// Display board: one task.
StaticTask_t display_task_tcb;
StackType_t display_task_stack[task_stack_words];

// TODO(lab, role 4): Add a stack and TCB for each of the three sensor tasks.

// This lab uses two boards playing different roles. Both roles live in this
// one file, and `this_device` below picks which tasks main() creates.
//
// For this lab, flip this by hand and rebuild before flashing each board.
// A more automated setup (e.g. a CI job that builds and flashes both roles
// from a single invocation) would instead read this from a compiler define
// passed on the build system's command line - something like
// `-DDEVICE_ROLE=DEVICE_ROLE_SENSOR` added to the project's build flags -
// rather than a hardcoded constant, so the role never has to be hand-edited
// in source.
enum class device_role : std::uint8_t
{
  sensor,
  display,
};

//constexpr device_role this_device = device_role::sensor;
}  // namespace

//constexpr (this_device == device_role::sensor)

static StaticTask_t adc_test_task_tcb;
static StackType_t adc_test_task_stack[task_stack_words];

void adc_test_task(void*);

int main()
{

  std::printf("Hello, World\n");

    // TODO(lab, role 4): Before the scheduler starts:
    //   1. Create sample_request and batch_queue.
    //   2. Create request_task, sampler_task, and transmit_task with
    //      xTaskCreateStatic().
    // request_task and transmit_task share one serial_driver. main()'s stack
    // is reused once the scheduler starts, so don't make it a local here.

  xTaskCreateStatic(
      adc_test_task,
      "adc_test_task",
      task_stack_words,
      nullptr,
      tskIDLE_PRIORITY + 1,
      adc_test_task_stack,
      &adc_test_task_tcb
  );

  vTaskStartScheduler();

  // Only reached if vTaskStartScheduler() itself fails to start. The
  // idle/timer task memory callbacks required by
  // configSUPPORT_STATIC_ALLOCATION are already supplied by the freertos_kernel
  // project's dpl/StaticAllocs_freertos.c - don't redefine them here.
  while (true) {
  }
}

//======================================================================
// TESTING
//======================================================================
void adc_test_task(void*)
{
    std::printf("ADC test task started!\n");

    std::printf("Creating ADC driver...\n");
    adc_driver adc;
    std::printf("ADC driver initialized!\n");

    std::array<std::uint16_t, 64> samples{};

    while (true)
    {
        std::printf("Starting ADC read...\n");

        adc.read(samples);

        if (!adc_read_success)
        {
          std::printf("ADC read failed. Skipping samples.\n");
          vTaskDelay(pdMS_TO_TICKS(2000));
          continue;
        }

        std::printf("ADC read completed!\n");

        for (std::size_t i = 0; i < samples.size(); i++)
        {
            std::printf("Sample %u: %u\n",
                static_cast<unsigned>(i),
                static_cast<unsigned>(samples[i]));
        }

        std::printf("ADC batch complete!\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
