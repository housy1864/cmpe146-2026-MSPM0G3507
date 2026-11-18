// cspell:ignore VDDA IIDX DDEVICE

#include <array>
#include <cstdint>
#include <cstdio>
#include <type_traits>
#include <cmath>
#include <cstring>
#include <span>

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
// The task that is currently waiting for the ADC/DMA transfer.
// ADC task waiting for the DMA-completion interrupt.
static TaskHandle_t adc_waiting_task = nullptr;
static volatile bool adc_read_success = false;

// ADC0 exposes a BD (basic-descriptor) DMA trigger. On MSPM0G3507,
// channels 0-2 are full channels, so use basic channel 3 for this trigger.
static constexpr uint8_t adc_dma_channel = 3;

class adc_driver : public lab5::adc16_stream
{
public:
    adc_driver()
    {
        // 1. Reset and power ADC0.
        DL_ADC12_reset(ADC0);
        DL_ADC12_enablePower(ADC0);
        delay_cycles(16000);

        // 2. Configure ADC clock to use SYSOSC.
        static const DL_ADC12_ClockConfig clock_config = {
            .clockSel = DL_ADC12_CLOCK_SYSOSC,
            .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
            .divideRatio = DL_ADC12_CLOCK_DIVIDE_1,
        };

        DL_ADC12_setClockConfig(ADC0, &clock_config);
        // Keep the ADC powered between conversions.
        DL_ADC12_setPowerDownMode(ADC0, DL_ADC12_POWER_DOWN_MODE_MANUAL);

        // 3. Configure repeated, software-triggered, 12-bit conversions.
        DL_ADC12_initSingleSample(
            ADC0,
            DL_ADC12_REPEAT_MODE_ENABLED,
            DL_ADC12_SAMPLING_SOURCE_MANUAL,
            DL_ADC12_TRIG_SRC_SOFTWARE,
            DL_ADC12_SAMP_CONV_RES_12_BIT,
            DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED
        );

        // 4. Configure MEM0 to read the internal temperature sensor.
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

        // 5. Configure the sample time.
        // Confirm this value represents >= 12.5 us with the configured ADC clock.
        DL_ADC12_setSampleTime0(ADC0, 400);

        // 6. Configure DMA completion interrupt.
        DL_ADC12_setDMASamplesCnt(ADC0, 1);
        DL_ADC12_enableDMATrigger(
            ADC0,
            DL_ADC12_DMA_MEM0_RESULT_LOADED
        );

        DL_ADC12_clearInterruptStatus(
            ADC0,
            DL_ADC12_INTERRUPT_DMA_DONE
        );

        DL_ADC12_enableInterrupt(
            ADC0,
            DL_ADC12_INTERRUPT_DMA_DONE
        );

        NVIC_EnableIRQ(ADC0_INT_IRQn);

        // 7. Configure DMA channel 0.
        static const DL_DMA_Config dma_config = {
            .trigger = DMA_ADC0_EVT_GEN_BD_TRIG,
            .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
            .transferMode = DL_DMA_SINGLE_TRANSFER_MODE,
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

        // ADC MEM0 result register is the fixed DMA source.
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
    // Convert 12 bits into 16 bits by repeating the most significant
    // four bits in the lower four positions.
    static uint16_t upscale_12_to_16(uint16_t value)
    {
        return static_cast<uint16_t>(
            (value << 4) | (value >> 8)
        );
    }

void driver_read(uint16_t* buffer, size_t length) override
{
    if (buffer == nullptr || length == 0 ||
        length > 0xFFFFU) {
        return;
    }

    DL_DMA_disableChannel(DMA, adc_dma_channel);

    DL_DMA_setDestAddr(
        DMA,
        adc_dma_channel,
        static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(buffer)
        )
    );

    DL_DMA_setTransferSize(
        DMA,
        adc_dma_channel,
        static_cast<uint16_t>(length)
    );

    ulTaskNotifyTake(pdTRUE, 0);

    DL_ADC12_clearInterruptStatus(
        ADC0,
        DL_ADC12_INTERRUPT_DMA_DONE
    );
    DL_ADC12_clearDMATriggerStatus(
        ADC0,
        DL_ADC12_DMA_MEM0_RESULT_LOADED
    );

    adc_read_success = false;
    adc_waiting_task = xTaskGetCurrentTaskHandle();

    DL_DMA_enableChannel(DMA, adc_dma_channel);
    DL_ADC12_enableDMA(ADC0);
    DL_ADC12_startConversion(ADC0);

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
        // A timeout means the ADC/DMA completion interrupt never arrived.
        DL_ADC12_stopConversion(ADC0);
        DL_ADC12_disableDMA(ADC0);
        DL_DMA_disableChannel(DMA, adc_dma_channel);
        adc_waiting_task = nullptr;

        // Capture ADC-side state so a failed DMA request can be diagnosed in CCS.
        const auto mem0_value =
            DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
        const auto dma_trigger_status =
            DL_ADC12_getRawDMATriggerStatus(
                ADC0, DL_ADC12_DMA_MEM0_RESULT_LOADED);
        const auto pending_interrupt =
            DL_ADC12_getPendingInterrupt(ADC0);
        std::printf(
            "ADC DMA timeout: MEM0=%u DMA trigger=0x%08lx pending IRQ=%u\n",
            static_cast<unsigned>(mem0_value),
            static_cast<unsigned long>(dma_trigger_status),
            static_cast<unsigned>(pending_interrupt));
        return;
    }

    adc_read_success = true;
    DL_ADC12_stopConversion(ADC0);
    DL_ADC12_disableDMA(ADC0);
    DL_DMA_disableChannel(DMA, adc_dma_channel);

    adc_waiting_task = nullptr;

    for (size_t i = 0; i < length; i++) {
        buffer[i] = upscale_12_to_16(buffer[i]);
    }
}
};



// TODO(lab, role 1): Store the handle of the task waiting in
// adc_driver::driver_read() somewhere this handler can reach. When
// DL_ADC12_getPendingInterrupt(ADC0) returns DL_ADC12_IIDX_DMA_DONE, wake that
// task with vTaskNotifyGiveFromISR() and finish with portYIELD_FROM_ISR().

extern "C" void ADC0_IRQHandler()
{
    if (DL_ADC12_getPendingInterrupt(ADC0) ==
        DL_ADC12_IIDX_DMA_DONE)
    {
        DL_ADC12_clearInterruptStatus(
            ADC0,
            DL_ADC12_INTERRUPT_DMA_DONE
        );

        BaseType_t higher_priority_task_woken = pdFALSE;

        if (adc_waiting_task != nullptr) {
            vTaskNotifyGiveFromISR(
                adc_waiting_task,
                &higher_priority_task_woken
            );
        }

        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
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
class uart_serial_driver : public lab6::serial
{ // Overview Page: 1,242
private:
  static constexpr std::size_t receive_buffer_size = 256;
  static inline std::array<std::uint8_t, receive_buffer_size> receive_storage{};
  static inline volatile std::size_t receive_write_cursor = 0;

  // UART1 uses PA8 for TX and PA9 for RX on the LaunchPad diagram.
  // PINCM19 and PINCM20 are entries 18 and 19 in the PINCM array.
  // Datasheet: Pages 11 for PINCMx, 23 for pin numbers,
  // IOMUX address on Datasheet Page 59.
  static constexpr std::uintptr_t IOMUX_PA8 = 0x4042'804CU;
  static constexpr std::uintptr_t IOMUX_PA9 = 0x4042'8050U;

  // IOMUX PINCM fields. PA8/PA9 use peripheral function 2 for UART1.
  static constexpr std::uint32_t PINCM_PF_UART1 = (0x2U << 0);
  static constexpr std::uint32_t PINCM_PC_CONNECTED = (1U << 7);
  static constexpr std::uint32_t PINCM_PIPU_ENABLE = (1U << 17);
  static constexpr std::uint32_t PINCM_INENA_ENABLE = (1U << 18);

  static volatile std::uint32_t* reg(std::uintptr_t address)
  {
    return reinterpret_cast<volatile std::uint32_t*>(address);
  }

public:
  static void handle_interrupt()
  {
    // Read UART1's interrupt index to identify the highest-priority pending event.
    switch (UART1->CPU_INT.IIDX) {
      // A receive interrupt means at least one byte is available in the RX FIFO.
      case UART_CPU_INT_IIDX_STAT_RXIFG:
      // A receive-timeout interrupt also means received data may remain in the FIFO.
      case UART_CPU_INT_IIDX_STAT_RTFG:
        // Continue until every byte currently waiting in the RX FIFO is processed.
        while ((UART1->STAT & UART_STAT_RXFE_MASK) ==
               UART_STAT_RXFE_CLEARED) {
          // Read the received byte and any error flags attached to that byte.
          std::uint32_t const received = UART1->RXDATA;
          // Combine all UART receive-error bits into one mask for validation.
          constexpr std::uint32_t error_mask =
            UART_RXDATA_FRMERR_MASK | UART_RXDATA_PARERR_MASK |
            UART_RXDATA_BRKERR_MASK | UART_RXDATA_OVRERR_MASK;
          // Store only bytes that were received without a UART-level error.
          if ((received & error_mask) == 0U) {
            // Keep only the 8-bit data field and write it at the current cursor.
            receive_storage[receive_write_cursor] =
              static_cast<std::uint8_t>(received & UART_RXDATA_DATA_MASK);
            // Advance the cursor and wrap it to zero at the buffer boundary.
            receive_write_cursor =
              (receive_write_cursor + 1U) % receive_buffer_size;
          }
        }
        // RXDATA reads acknowledge the received bytes and leave the handler.
        break;

      // These interrupt causes indicate a UART receive error.
      case UART_CPU_INT_IIDX_STAT_FEFG:
      case UART_CPU_INT_IIDX_STAT_PEFG:
      case UART_CPU_INT_IIDX_STAT_BEFG:
      case UART_CPU_INT_IIDX_STAT_OEFG:
        // If an errored byte is present, read it to remove it from the FIFO.
        if ((UART1->STAT & UART_STAT_RXFE_MASK) ==
            UART_STAT_RXFE_CLEARED) {
          (void)UART1->RXDATA;
        }
        // Explicitly clear all receive-error interrupt flags.
        UART1->CPU_INT.ICLR = UART_CPU_INT_ICLR_FRMERR_CLR |
                              UART_CPU_INT_ICLR_PARERR_CLR |
                              UART_CPU_INT_ICLR_BRKERR_CLR |
                              UART_CPU_INT_ICLR_OVRERR_CLR;
        // Do not place an errored byte into the receive buffer.
        break;

      // Ignore interrupt causes that are not part of this driver yet.
      default:
        break;
    }
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    // UART1 uses the 32 MHz BUSCLK on the MSPM0G3507.  Keep this value here
    // until the driver grows a configurable clock-source selection.
    constexpr std::uint32_t uart_clock_hz = 32'000'000U;
    constexpr std::uint32_t oversampling = 16U;

    if (p_settings.baud_rate == 0U) {
      return false;
    }

    /* With 16x oversampling, the UART divisor is represented as:

         divisor = IBRD + FBRD / 64

      Round the fractional part to the nearest representable value.  Do all
      validation before touching the peripheral registers.

      Purpose of a divisor: Slows down the clock by a specific rate to attain
      the baud rate desired for the UART connection.

    */
    // Oversampling: Pg. 1,247. Baud rate generation: Pg. 1,248
    std::uint32_t const divisor_x64 =
      (uart_clock_hz * 64U + (oversampling * p_settings.baud_rate) / 2U) /
      (oversampling * p_settings.baud_rate);
    std::uint32_t const integer_divisor = divisor_x64 / 64U;
    std::uint32_t const fractional_divisor = divisor_x64 % 64U;

    // If Divisor Value out of range or 0, return false as a fail-safe.
    if (integer_divisor == 0U || integer_divisor > 0xFFFFU) {
      return false;
    }

    std::uint32_t parity = UART_LCRH_PEN_DISABLE;
    switch (p_settings.parity) {
      case settings::parity::none:
        break; // No parity bit transmitted
      case settings::parity::odd:
        parity = UART_LCRH_PEN_ENABLE | UART_LCRH_EPS_ODD;
        break; // Parity with odd number of 1 bits
      case settings::parity::even:
        parity = UART_LCRH_PEN_ENABLE | UART_LCRH_EPS_EVEN;
        break; // Parity with even number of 1 bits
      case settings::parity::forced1:
        parity = UART_LCRH_PEN_ENABLE | UART_LCRH_SPS_ENABLE |
                 UART_LCRH_EPS_ODD;
        break; // Parity bit always 1
      case settings::parity::forced0:
        parity = UART_LCRH_PEN_ENABLE | UART_LCRH_SPS_ENABLE |
                 UART_LCRH_EPS_EVEN;
        break; // Parity bit always 0
    }

    // The LaunchPad diagram routes this UART through PA8 (TX) and PA9 (RX).
    // J14 must be installed in position 2-3 so the header RX signal selects
    // PA9 rather than PB23.
    *reg(IOMUX_PA8) = PINCM_PF_UART1 | PINCM_PC_CONNECTED;
    *reg(IOMUX_PA9) = PINCM_PF_UART1 | PINCM_PC_CONNECTED |
                      PINCM_INENA_ENABLE | PINCM_PIPU_ENABLE;

    // Register-level UART1 configuration.
    UART1->GPRCM.PWREN =
      UART_PWREN_KEY_UNLOCK_W | UART_PWREN_ENABLE_ENABLE; // Pg. 1267
    UART1->GPRCM.RSTCTL =
      UART_RSTCTL_KEY_UNLOCK_W | UART_RSTCTL_RESETSTKYCLR_CLR |
      UART_RSTCTL_RESETASSERT_ASSERT; // Pg. 1268
    UART1->GPRCM.RSTCTL =
      UART_RSTCTL_KEY_UNLOCK_W | UART_RSTCTL_RESETSTKYCLR_CLR;

    UART1->CLKSEL = UART_CLKSEL_BUSCLK_SEL_ENABLE; // Pg. 1272
    UART1->CLKDIV = UART_CLKDIV_RATIO_DIV_BY_1; // Clock divider not used, dividing only done for Baud rate

    UART1->CTL0 = UART_CTL0_MODE_UART | UART_CTL0_HSE_OVS16 |
                  UART_CTL0_TXE_ENABLE | UART_CTL0_RXE_ENABLE |
                  UART_CTL0_FEN_ENABLE; // Pgs 1299-1301
    UART1->LCRH = UART_LCRH_WLEN_DATABIT8 | parity |
                  (p_settings.stop == settings::stop_bits::two
                     ? UART_LCRH_STP2_ENABLE
                     : UART_LCRH_STP2_DISABLE); // Pgs. 1302-1303
    UART1->IBRD = integer_divisor; // Pg. 1308
    UART1->FBRD = fractional_divisor; // Pg. 1309

    // The UART requires an LCRH write after changing IBRD/FBRD.  Rewriting
    // the complete line-control value also makes the ordering explicit.
    UART1->LCRH = UART_LCRH_WLEN_DATABIT8 | parity |
                  (p_settings.stop == settings::stop_bits::two
                     ? UART_LCRH_STP2_ENABLE
                     : UART_LCRH_STP2_DISABLE);
    UART1->CTL0 |= UART_CTL0_ENABLE_ENABLE;

    UART1->IFLS = (UART1->IFLS & ~UART_IFLS_RXIFLSEL_MASK) |
                  UART_IFLS_RXIFLSEL_LVL_1;
    UART1->CPU_INT.IMASK = UART_CPU_INT_IMASK_RXINT_SET |
                           UART_CPU_INT_IMASK_FRMERR_SET |
                           UART_CPU_INT_IMASK_PARERR_SET |
                           UART_CPU_INT_IMASK_BRKERR_SET |
                           UART_CPU_INT_IMASK_OVRERR_SET;
    NVIC_EnableIRQ(UART1_INT_IRQn);
    return true;
  }

  void driver_write(std::span<std::uint8_t const> p_data) override
  {
    for (std::uint8_t const byte : p_data) {
      while ((UART1->STAT & UART_STAT_TXFF_MASK) !=
             UART_STAT_TXFF_CLEARED) {
      }
      UART1->TXDATA = byte; // Simply lists the byte UART tx will send
    }
  }

  std::span<std::uint8_t const> driver_receive_buffer() override
  {
    return {receive_storage.data(), receive_storage.size()};
  }

  std::size_t driver_receive_cursor() override
  {
    return receive_write_cursor;
  }
};

extern "C" void UART1_IRQHandler()
{
  uart_serial_driver::handle_interrupt();
}


// =============================================================================
// Role 3: CRC
// =============================================================================

// TODO(lab, role 3): Make this class inherit from and implement the
// `lab6::crc32` interface (`crc.hpp`), written register-level against the
// on-chip CRC accelerator - no DriverLib. Its output must match
// crc32_software below for the same input. Check the accelerator's bit-order
// options and whether it applies the final XOR.
class crc32_hardware : public lab6::crc32
{
private:

  static constexpr std::uintptr_t CRC_BASE = 0x40440000;

  static constexpr std::uintptr_t CRC_PWREN = CRC_BASE + 0x0800;
  static constexpr std::uintptr_t CRC_CTRL  = CRC_BASE + 0x1100;
  static constexpr std::uintptr_t CRC_SEED  = CRC_BASE + 0x1104;
  static constexpr std::uintptr_t CRC_IN    = CRC_BASE + 0x1108;
  static constexpr std::uintptr_t CRC_OUT   = CRC_BASE + 0x111C;

  static constexpr std::uint32_t INITIAL_VALUE = 0xFFFFFFFFU;
  static constexpr std::uint32_t FINAL_XOR     = 0xFFFFFFFFU;
  static constexpr std::uint32_t BITREVERSE   = (1U << 1);

  static volatile std::uint32_t& reg32(std::uintptr_t address)
  {
    return *reinterpret_cast<volatile std::uint32_t*>(address);
  }

  static volatile std::uint8_t& reg8(std::uintptr_t address)
  {
    return *reinterpret_cast<volatile std::uint8_t*>(address);
  }

  void driver_reset() override
  {
    // Enable CRC peripheral.
    reg32(CRC_PWREN) = ((0x26U << 24) | 0x01U);

    // Configure CRC32-ISO3309.
    reg32(CRC_CTRL) = BITREVERSE;

    // Initialize CRC seed.
    reg32(CRC_SEED) = INITIAL_VALUE;
  }

  void driver_update(std::span<std::uint8_t const> p_data) override
  {
    for (auto const byte : p_data)
    {
      reg8(CRC_IN) = byte;
    }
  }

  std::uint32_t driver_checksum() override
  {
    std::uint32_t result = reg32(CRC_OUT);

    return result ^ FINAL_XOR;
  }

public:

  crc32_hardware()
  {
    reset();
  }
};

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

static temperature_sample_t convert_temperature(
    std::uint16_t raw_sample,
    std::uint16_t calibration_code)
{
    // Convert the upscaled 16-bit sample back to a 12-bit ADC code.
    const double sample_code = raw_sample >> 4;

    // Convert the ADC code difference into voltage.
    const double voltage_difference =
        (sample_code - calibration_code) * 3.3 / 4096.0;

    // Temperature sensor slope is -1.8 mV per degree Celsius.
    const double temperature =
        30.0 + voltage_difference / -0.0018;

    // Store temperature in units of 0.1 degrees Celsius.
    return static_cast<temperature_sample_t>(
        std::lround(temperature * 10.0)
    );
}

// A packet on the wire: the batch's bytes, followed by the 4-byte CRC-32.
constexpr std::size_t packet_size =
  sizeof(temperature_batch) + sizeof(std::uint32_t);

// =============================================================================
// Sensor board: semaphore and queue (role 4 creates these in main())
// =============================================================================

// request_task -> sampler_task: "take a batch now". Carries no data.
// Role 4: static backing storage; main() creates the binary semaphore.
StaticSemaphore_t sample_request_control_block;
SemaphoreHandle_t sample_request = nullptr;

// sampler_task -> transmit_task: a copy of one temperature_batch.
// Role 4: one queue item is a full copy of the converted sample batch.
StaticQueue_t batch_queue_control_block;
std::array<std::uint8_t, sizeof(temperature_batch)> batch_queue_storage;
QueueHandle_t batch_queue = nullptr;

// =============================================================================
// Role 4: shared serial object and application helpers
// =============================================================================

// Provisional protocol for the application: signed 0.1 C samples and CRC,
// both little-endian. Confirm this with the ADC and transmit-task owners.
constexpr std::size_t sample_bytes = sizeof(temperature_batch);
using packet = std::array<std::uint8_t, packet_size>;
static_assert(packet_size == 132);
constexpr TickType_t poll_ticks = pdMS_TO_TICKS(2);
constexpr TickType_t debounce_ticks = pdMS_TO_TICKS(20);
constexpr TickType_t response_timeout_ticks = pdMS_TO_TICKS(500);
static_assert(poll_ticks > 0 && debounce_ticks > 0);

// Temporary stand-ins allowed by the README. Change these switches as each
// teammate finishes their implementation. No hardware driver is implemented here.
constexpr bool use_uart_stub = false;
constexpr bool use_software_crc = false;
constexpr bool sampler_ready = true;
constexpr bool transmitter_ready = true;

class application_serial_stub : public lab6::serial
{
private:
  bool driver_configure(settings const&) override
  {
    return true;
  }

  void driver_write(std::span<std::uint8_t const> p_data) override
  {
    std::printf("UART stub: write %u bytes\n",
                static_cast<unsigned>(p_data.size()));
  }

  std::span<std::uint8_t const> driver_receive_buffer() override
  {
    return { m_buffer.data(), m_buffer.size() };
  }

  std::size_t driver_receive_cursor() override
  {
    return 0;
  }

  std::array<std::uint8_t, 256> m_buffer{};
};

using application_serial = uart_serial_driver;
using application_crc = crc32_hardware;

// Role 3's transmit_task writes through this same object. Static storage keeps
// it alive after FreeRTOS reuses main()'s stack.
application_serial board_serial;
std::size_t initial_receive_cursor = 0;

enum class led_color
{
  off,
  blue,
  yellow,
  red,
  magenta,
};

std::size_t read_available(lab6::serial& p_serial,
                           std::size_t& p_cursor,
                           std::span<std::uint8_t> p_destination)
{
  auto const buffer = p_serial.receive_buffer();
  auto const head = p_serial.receive_cursor();
  std::size_t copied = 0;
  while (p_cursor != head && copied < p_destination.size()) {
    p_destination[copied++] = buffer[p_cursor];
    p_cursor = (p_cursor + 1) % buffer.size();
  }
  return copied;
}

// Clock/delay callbacks use FreeRTOS on the board and simulated time in tests.
// Unsigned tick subtraction also handles the tick counter wrapping around.
template<typename Tick, typename Now, typename Sleep>
bool receive_packet(lab6::serial& p_serial,
                    std::size_t p_cursor,
                    packet& p_packet,
                    Tick p_started,
                    Tick p_timeout,
                    Now p_now,
                    Sleep p_sleep)
{
  std::size_t received = 0;
  while (static_cast<Tick>(p_now() - p_started) < p_timeout) {
    received += read_available(
      p_serial, p_cursor,
      { p_packet.data() + received, p_packet.size() - received });
    if (received == p_packet.size()) {
      return true;
    }
    p_sleep();
  }
  return false;
}

bool valid_crc(packet const& p_packet, lab6::crc32& p_crc)
{
  std::uint32_t received = 0;
  for (std::size_t byte = 0; byte < sizeof(received); ++byte) {
    received |= static_cast<std::uint32_t>(p_packet[sample_bytes + byte])
                << (8 * byte);
  }
  p_crc.reset();
  p_crc.update({ p_packet.data(), sample_bytes });
  return p_crc.checksum() == received;
}

led_color temperature_color(packet const& p_packet)
{
  std::int32_t sum = 0;
  for (std::size_t sample = 0; sample < samples_per_batch; ++sample) {
    auto const low = static_cast<std::uint32_t>(p_packet[2 * sample]);
    auto const high = static_cast<std::uint32_t>(p_packet[2 * sample + 1]);
    auto const encoded = low | (high << 8);
    // Decode signed samples explicitly; avoid unaligned pointer casts.
    sum += encoded < 0x8000U ? static_cast<std::int32_t>(encoded)
                            : static_cast<std::int32_t>(encoded) - 0x10000;
  }
  // Compare sums to avoid rounding the average across a color boundary.
  constexpr std::int32_t batch_count = samples_per_batch;
  if (sum < 250 * batch_count) {
    return led_color::blue;
  }
  if (sum < 300 * batch_count) {
    return led_color::yellow;
  }
  return led_color::red;
}

void set_led(lab1::output_pin& p_red,
             lab1::output_pin& p_green,
             lab1::output_pin& p_blue,
             led_color p_color)
{
  p_red.level(p_color == led_color::red || p_color == led_color::yellow ||
              p_color == led_color::magenta);
  p_green.level(p_color == led_color::yellow);
  p_blue.level(p_color == led_color::blue || p_color == led_color::magenta);
}

// Match Lab 1's wiring: S1 is active high; S2 is active low.
bool button_pressed(lab1::input_pin& p_s1, lab1::input_pin& p_s2)
{
  return p_s1.level() || !p_s2.level();
}

void wait_for_button_state(lab1::input_pin& p_s1,
                           lab1::input_pin& p_s2,
                           bool p_pressed)
{
  while (true) {
    while (button_pressed(p_s1, p_s2) != p_pressed) {
      vTaskDelay(poll_ticks);
    }
    auto const started = xTaskGetTickCount();
    while (button_pressed(p_s1, p_s2) == p_pressed) {
      if (static_cast<TickType_t>(xTaskGetTickCount() - started) >=
          debounce_ticks) {
        return;
      }
      vTaskDelay(poll_ticks);
    }
  }
}

// Sleep instead of running the teammates' empty infinite loops until ready.
void pending_sensor_task(void*)
{
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void poll_requests(lab6::serial& p_serial, std::size_t& p_cursor)
{
  std::array<std::uint8_t, 64> incoming;
  auto const count = read_available(
    p_serial, p_cursor, { incoming.data(), incoming.size() });
  for (std::size_t index = 0; index < count; ++index) {
    if (incoming[index] == sample_request_command) {
      // A binary semaphore intentionally combines repeated pending requests.
      xSemaphoreGive(sample_request);
    }
  }
}

// =============================================================================
// Sensor board: tasks
// =============================================================================

// Role 4: Every few ms (vTaskDelay()), compare the serial driver's
// receive_cursor() with the last position you read. When
// sample_request_command arrives, xSemaphoreGive(sample_request). This task
// only reads from the serial driver.
void request_task(void*)
{
  auto cursor = initial_receive_cursor;
  while (true) {
    poll_requests(board_serial, cursor);
    vTaskDelay(poll_ticks);
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
  adc_driver adc;
  std::array<std::uint16_t, samples_per_batch> raw_samples{};
  temperature_batch temperatures{};

  // Factory calibration value at 30 degrees Celsius.
  const std::uint16_t calibration_code =
    DL_SYSCTL_getTempCalibrationConstant();

  while (true) {
    xSemaphoreTake(sample_request, portMAX_DELAY);

    // One ADC DMA transfer fills the entire raw sample array.
    adc_read_success = false;
    adc.read(raw_samples.data(), raw_samples.size());

    if (!adc_read_success) {
      std::printf("ADC DMA transfer failed; dropping this batch\n");
      continue;
    }

    for (std::size_t i = 0; i < samples_per_batch; ++i) {
      temperatures[i] =
        convert_temperature(raw_samples[i], calibration_code);
    }

    if (xQueueSend(batch_queue, &temperatures, portMAX_DELAY) != pdPASS) {
      std::printf("Could not queue ADC temperature batch\n");
    }
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
  application_crc crc;
  crc32_software software_crc;
  // S1 is active high on the LaunchPad. Holding it deliberately corrupts a packet.
  lab1::mspm0_input_pin corruption_button(
    lab1::gpioa_base, 18, 40, lab1::pin_resistor::pull_down);

  temperature_batch batch{};

  while (true) {
    if (xQueueReceive(batch_queue, &batch, portMAX_DELAY) != pdPASS) {
      continue;
    }

    packet outgoing{};

    // The batch is a contiguous array of signed 0.1 C samples.
    std::memcpy(outgoing.data(), batch.data(), sample_bytes);

    crc.reset();
    crc.update({outgoing.data(), sample_bytes});
    const std::uint32_t checksum = crc.checksum();

    software_crc.reset();
    software_crc.update({outgoing.data(), sample_bytes});
    if (software_crc.checksum() != checksum) {
      std::printf("WARNING: hardware CRC does not match software CRC\n");
    }

    // Test hook: alter one data bit after computing CRC, so the receiver detects it.
    if (corruption_button.level()) {
      outgoing[0] ^= 0x01U;
      std::printf("Corruption test: flipped one packet bit\n");
    }

    // Append CRC in little-endian byte order.
    for (std::size_t byte = 0; byte < sizeof(checksum); ++byte) {
      outgoing[sample_bytes + byte] =
        static_cast<std::uint8_t>((checksum >> (8U * byte)) & 0xFFU);
    }

    board_serial.write({outgoing.data(), outgoing.size()});
  }
}
// =============================================================================
// Display board: task
// =============================================================================

// Role 4: Use the shared serial driver, a CRC implementation, two
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
  application_crc crc;
  lab1::mspm0_input_pin s1(lab1::gpioa_base, 18, 40,
                           lab1::pin_resistor::pull_down);
  lab1::mspm0_input_pin s2(lab1::gpiob_base, 21, 49,
                           lab1::pin_resistor::pull_up);
  lab1::mspm0_output_pin red(lab1::gpiob_base, 26, 57);
  lab1::mspm0_output_pin green(lab1::gpiob_base, 27, 58);
  lab1::mspm0_output_pin blue(lab1::gpiob_base, 22, 50);
  set_led(red, green, blue, led_color::off);

  while (true) {
    // Require a stable release and press: holding a button sends one request.
    wait_for_button_state(s1, s2, false);
    wait_for_button_state(s1, s2, true);

    // Discard stale bytes from before the request, including partial replies.
    auto const cursor = board_serial.receive_cursor();
    auto const started = xTaskGetTickCount();
    board_serial.write({ &sample_request_command, 1 });

    packet response{};
    bool const received = receive_packet(
      board_serial, cursor, response, started, response_timeout_ticks,
      [] { return xTaskGetTickCount(); },
      [] { vTaskDelay(poll_ticks); });
    if (!received) {
      std::printf("Temperature response timeout (500 ms)\n");
      set_led(red, green, blue, led_color::off);
      continue;
    }
    if (!valid_crc(response, crc)) {
      std::printf("Temperature packet dropped: CRC mismatch\n");
      set_led(red, green, blue, led_color::magenta);
      continue;
    }
    set_led(red, green, blue, temperature_color(response));
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

// Role 4: Each sensor task owns its static stack and task control block.
StaticTask_t request_task_tcb;
StackType_t request_task_stack[task_stack_words];
StaticTask_t sampler_task_tcb;
StackType_t sampler_task_stack[task_stack_words];
StaticTask_t transmit_task_tcb;
StackType_t transmit_task_stack[task_stack_words];

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

constexpr device_role this_device = device_role::sensor;
}  // namespace

int main()
{
  std::printf("Lab 5 application\n");
  if constexpr (use_uart_stub) {
    std::printf("UART stub enabled: no board-to-board communication\n");
  }
  if constexpr (use_software_crc) {
    std::printf("Using the supplied software CRC\n");
  }

  // Role 4: Configure once, before the shared serial object is used by tasks.
  // Defaults are 115200 baud, 8 data bits, no parity, one stop bit.
  if (!board_serial.configure(lab6::serial::settings{}) ||
      board_serial.receive_buffer().size() <= packet_size) {
    std::printf("Serial setup failed or receive buffer is too small\n");
    return 1;
  }
  initial_receive_cursor = board_serial.receive_cursor();

  bool tasks_created = false;
  if constexpr (this_device == device_role::sensor) {
    // Role 4: Create synchronization objects before any task can access them.
    sample_request = xSemaphoreCreateBinaryStatic(&sample_request_control_block);
    batch_queue = xQueueCreateStatic(
      1, sizeof(temperature_batch), batch_queue_storage.data(),
      &batch_queue_control_block);
    if (sample_request == nullptr || batch_queue == nullptr) {
      std::printf("Sensor semaphore/queue creation failed\n");
      return 1;
    }

    if constexpr (!sampler_ready || !transmitter_ready) {
      std::printf("Unfinished sensor tasks use sleeping placeholders\n");
    }
    auto const request_handle = xTaskCreateStatic(
      request_task, "request", task_stack_words, nullptr,
      tskIDLE_PRIORITY + 1, request_task_stack, &request_task_tcb);
    auto const sampler_handle = xTaskCreateStatic(
      sampler_ready ? sampler_task : pending_sensor_task,
      "sampler", task_stack_words, nullptr, tskIDLE_PRIORITY + 1,
      sampler_task_stack, &sampler_task_tcb);
    auto const transmit_handle = xTaskCreateStatic(
      transmitter_ready ? transmit_task : pending_sensor_task,
      "transmit", task_stack_words, nullptr, tskIDLE_PRIORITY + 1,
      transmit_task_stack, &transmit_task_tcb);
    tasks_created = request_handle != nullptr && sampler_handle != nullptr &&
                    transmit_handle != nullptr;
  } else {
    tasks_created = xTaskCreateStatic(
      display_task, "display", task_stack_words, nullptr,
      tskIDLE_PRIORITY + 1, display_task_stack, &display_task_tcb) != nullptr;
  }

  if (!tasks_created) {
    std::printf("Application task creation failed\n");
    return 1;
  }
  vTaskStartScheduler();
  std::printf("FreeRTOS scheduler failed to start\n");
  return 1;
}