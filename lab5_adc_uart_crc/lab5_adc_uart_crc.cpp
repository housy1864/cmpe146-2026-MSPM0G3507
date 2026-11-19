// cspell:ignore VDDA IIDX DDEVICE

#include <array>
#include <cstdint>
#include <cstdio>
#include <limits>

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
class adc_driver
{};

// TODO(lab, role 1): Store the handle of the task waiting in
// adc_driver::driver_read() somewhere this handler can reach. When
// DL_ADC12_getPendingInterrupt(ADC0) returns DL_ADC12_IIDX_DMA_DONE, wake that
// task with vTaskNotifyGiveFromISR() and finish with portYIELD_FROM_ISR().
extern "C" void ADC0_IRQHandler()
{}

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
  static inline std::array<std::uint8_t, receive_buffer_size> receive_storage{}; // Used to store data at cursor location.
  static inline volatile std::size_t receive_write_cursor = 0; 
  // The cursor is used to find the next valid received byte based on the buffer.

  // UART1 uses PA8 for TX and PA9 for RX on the LaunchPad diagram.
  // PINCM19 and PINCM20 are entries 18 and 19 in the PINCM array.
  // Datasheet: Pages 11 for PINCMx, 23 for pin numbers,
  // IOMUX address on Datasheet Page 59.
  static constexpr std::uintptr_t IOMUX_PA8 = 0x4042'804CU;
  static constexpr std::uintptr_t IOMUX_PA9 = 0x4042'8050U;

  // UART1 register addresses used by this class. TRM Pgs. 1265-1266
  static constexpr std::uintptr_t UART1_REG_BASE = 0x4010'0000U; // Datasheet Pg. 59
  static constexpr std::uintptr_t UART1_PWREN = UART1_REG_BASE + 0x800U;
  static constexpr std::uintptr_t UART1_RSTCTL = UART1_REG_BASE + 0x804U;
  static constexpr std::uintptr_t UART1_CLKDIV = UART1_REG_BASE + 0x1000U;
  static constexpr std::uintptr_t UART1_CLKSEL = UART1_REG_BASE + 0x1008U;
  static constexpr std::uintptr_t UART1_IIDX = UART1_REG_BASE + 0x1020U;
  static constexpr std::uintptr_t UART1_IMASK = UART1_REG_BASE + 0x1028U;
  static constexpr std::uintptr_t UART1_ICLR = UART1_REG_BASE + 0x1048U;
  static constexpr std::uintptr_t UART1_CTL0 = UART1_REG_BASE + 0x1100U;
  static constexpr std::uintptr_t UART1_LCRH = UART1_REG_BASE + 0x1104U;
  static constexpr std::uintptr_t UART1_STAT = UART1_REG_BASE + 0x1108U;
  static constexpr std::uintptr_t UART1_IFLS = UART1_REG_BASE + 0x110CU;
  static constexpr std::uintptr_t UART1_IBRD = UART1_REG_BASE + 0x1110U;
  static constexpr std::uintptr_t UART1_FBRD = UART1_REG_BASE + 0x1114U;
  static constexpr std::uintptr_t UART1_TXDATA = UART1_REG_BASE + 0x1120U;
  static constexpr std::uintptr_t UART1_RXDATA = UART1_REG_BASE + 0x1124U;
  static constexpr std::uintptr_t NVIC_ISER0 = 0xE000'E100U;

  // UART configuration values used by this class.
  // Unlocks writes to the UART power-control register.
  static constexpr std::uint32_t PWREN_KEY = 0x2600'0000U;
  // Powers the UART peripheral on.
  static constexpr std::uint32_t PWREN_ENABLE = (1U << 0U);
  // Unlocks writes to the UART reset-control register.
  static constexpr std::uint32_t RSTCTL_KEY = 0xB100'0000U;
  // Clears a previously latched UART reset request.
  static constexpr std::uint32_t RSTCTL_STICKY_CLEAR = (1U << 1U);
  // Asserts reset so configuration starts from a known state.
  static constexpr std::uint32_t RSTCTL_ASSERT = (1U << 0U);
  // Selects the 32 MHz BUSCLK as UART1's clock source.
  static constexpr std::uint32_t CLKSEL_BUSCLK = (1U << 3U);
  // Enables the UART after all other configuration is complete.
  static constexpr std::uint32_t CTL0_ENABLE = (1U << 0U);
  // Enables the UART receiver.
  static constexpr std::uint32_t CTL0_RXE = (1U << 3U);
  // Enables the UART transmitter.
  static constexpr std::uint32_t CTL0_TXE = (1U << 4U);
  // Enables the UART FIFOs.
  static constexpr std::uint32_t CTL0_FEN = (1U << 17U);
  // Enables parity generation/checking when selected by settings.
  // CTL ENABLE VALUES: TRM Pg. 1299
  static constexpr std::uint32_t LCRH_PEN = (1U << 1U);
  // Selects even parity when set; clear selects odd parity.
  static constexpr std::uint32_t LCRH_EPS = (1U << 2U);
  // Selects two stop bits; clear selects one stop bit.
  static constexpr std::uint32_t LCRH_STP2 = (1U << 3U);
  // Selects eight data bits per UART frame.
  static constexpr std::uint32_t LCRH_WLEN8 = 3U << 5U;
  // Forces the selected parity value instead of calculating parity.
  // LINE CONTROL REGISTER VALUES: Pg. 1302
  static constexpr std::uint32_t LCRH_SPS = (1U << 7U);
  // Identifies the RX FIFO threshold field to update.
  static constexpr std::uint32_t IFLS_RX_MASK = 7U << 4U;
  // Requests an RX interrupt at the lowest FIFO threshold.
  static constexpr std::uint32_t IFLS_RX_LEVEL_1 = 7U << 4U;
  // Enables the UART receive-data interrupt.
  static constexpr std::uint32_t INT_RX = (1U << 10U);
  // Enables framing-error interrupts.
  static constexpr std::uint32_t INT_FRMERR = (1U << 1U);
  // Enables parity-error interrupts.
  static constexpr std::uint32_t INT_PARERR = (1U << 2U);
  // Enables break-error interrupts.
  static constexpr std::uint32_t INT_BRKERR = (1U << 3U);
  // Enables receive-overrun interrupts.
  static constexpr std::uint32_t INT_OVRERR = (1U << 4U);
  // Interrupt index for receive timeout.
  static constexpr std::uint32_t IIDX_RTFG = 1U;
  // Interrupt index for framing error.
  static constexpr std::uint32_t IIDX_FEFG = 2U;
  // Interrupt index for parity error.
  static constexpr std::uint32_t IIDX_PEFG = 3U;
  // Interrupt index for break error.
  static constexpr std::uint32_t IIDX_BEFG = 4U;
  // Interrupt index for receive overrun.
  static constexpr std::uint32_t IIDX_OEFG = 5U;
  // Interrupt index for received data available.
  static constexpr std::uint32_t IIDX_RXIFG = 0xBU;
  // Indicates that the RX FIFO is empty.
  static constexpr std::uint32_t STAT_RXFE = (1U << 2U);
  // Indicates that the TX FIFO is full.
  static constexpr std::uint32_t STAT_TXFF = (1U << 7U);
  // Stat flag info: TRM Page. 1250
  // Selects the eight-bit data field from RXDATA.
  static constexpr std::uint32_t RXDATA_MASK = 0xFFU;
  // Marks a framing error on the received byte.
  static constexpr std::uint32_t RXDATA_FRMERR = (1U << 8U);
  // Marks a parity error on the received byte.
  static constexpr std::uint32_t RXDATA_PARERR = (1U << 9U);
  // Marks a break condition on the received byte.
  static constexpr std::uint32_t RXDATA_BRKERR = (1U << 10U);
  // Marks a receive-overrun error on the received byte.
  static constexpr std::uint32_t RXDATA_OVRERR = (1U << 11U);
  // RXDATA ERROR Values: Pg. 1262

  // Select UART1's peripheral function for PA8 (TX) and PA9 (RX).
  // Digital Pin Value: Datasheet Pg. 11
  static constexpr std::uint32_t PINCM_PF_UART1 = 2U;

  // Connect the PINCM-controlled pin to its selected peripheral function.
  static constexpr std::uint32_t PINCM_PC_CONNECTED = (1U << 7);

  // Enable the internal pull-up used by the idle-high UART RX line (PA9).
  static constexpr std::uint32_t PINCM_PIPU_ENABLE = (1U << 17);

  // Enable the input buffer so UART1 can sample PA9.
  static constexpr std::uint32_t PINCM_INENA_ENABLE = (1U << 18);

  static volatile std::uint32_t* reg(std::uintptr_t address)
  {
    return reinterpret_cast<volatile std::uint32_t*>(address);
  }

  static volatile std::uint32_t* uart_reg(std::uintptr_t const address)
  {
    return reg(address);
  }

public:
  explicit uart_serial_driver(settings const& p_settings)
    : m_configured(driver_configure(p_settings))
  {
  } // Configures UART at construction, is following 1-Phase Initialization

  [[nodiscard]] bool is_configured() const
  {
    return m_configured;
  }

  struct debug_registers {
    std::uint32_t stat;
    std::uint32_t iidx;
    std::uint32_t ctl0;
    std::uint32_t clksel;
    std::uint32_t pwren;
  };

  [[nodiscard]] static debug_registers debug_state()
  {
    return { *uart_reg(UART1_STAT), *uart_reg(UART1_IIDX),
             *uart_reg(UART1_CTL0), *uart_reg(UART1_CLKSEL),
             *uart_reg(UART1_PWREN) };
  } // Reads current register values. Used for debugging purposes in main to make sure
    // Register values are working and that the UART driver works as intended.

  static void handle_interrupt()
  {
    // Read UART1's interrupt index to identify the highest-priority pending event.
    switch (*uart_reg(UART1_IIDX)) {
      // A receive interrupt means at least one byte is available in the RX FIFO.
      case IIDX_RXIFG:
      // A receive-timeout interrupt also means received data may remain in the FIFO.
      case IIDX_RTFG:
        // Processes as long as a byte is available in the queue to process.
        while ((*uart_reg(UART1_STAT) & STAT_RXFE) == 0U) {
          // The byte is received during UART transfer from TX to RX.
          // RXFE Page for FIFO full flagging: TRM Pg. 1250
          std::uint32_t const received = *uart_reg(UART1_RXDATA);
          // Combine all UART receive-error bits into one mask for validation.
          constexpr std::uint32_t error_mask =
            RXDATA_FRMERR | RXDATA_PARERR | RXDATA_BRKERR | RXDATA_OVRERR;
          // Checks to see if error is not received. Otherwise, breaks interrupt
          // Flag Registers: TRM Pg. 1249
          if ((received & error_mask) == 0U) {
            receive_storage[receive_write_cursor] =
              static_cast<std::uint8_t>(received & RXDATA_MASK); // Stores a received byte
              // at the cursor location to the buffer.
            // Advance the cursor and wrap it to zero at the buffer boundary.
            receive_write_cursor =
              (receive_write_cursor + 1U) % receive_buffer_size; 
              // Cursor incremented according to current position based on the buffer size
          }
        }
        // RXDATA reads acknowledge the received bytes and leave the handler.
        break;

      // These interrupt causes indicate a UART receive error. Pg. 1274
      case IIDX_FEFG:
      case IIDX_PEFG:
      case IIDX_BEFG:
      case IIDX_OEFG: // During an overrun error
        // If an errored byte is present, read it to remove it from the FIFO.
        if ((*uart_reg(UART1_STAT) & STAT_RXFE) == 0U) {
          (void)*uart_reg(UART1_RXDATA);
        }
        // Explicitly clear all receive-error interrupt flags.
        *uart_reg(UART1_ICLR) = INT_FRMERR | INT_PARERR |
                                INT_BRKERR | INT_OVRERR;
        // Do not place an errored byte into the receive buffer.
        break;

      // Ignore interrupt causes that are not part of this driver yet.
      default:
        break;
    }
  }

private:
  bool m_configured;

  bool driver_configure(settings const& p_settings) override
  {
    // UART1 uses the 32 MHz BUSCLK on the MSPM0G3507.  Keep this value here
    // until the driver grows a configurable clock-source selection.
    constexpr std::uint32_t uart_clock_hz = 32'000'000U;
    constexpr std::uint32_t oversampling = 16U;
    // Oversampling being 16 means that each bit period is divided into 16 sample intervals
    // Supported oversampling rates: TRM Pg. 1243
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
    // OVERSAMPLING Photo: Pg. 1,247. BAUD RATE Generation Equation: Pg. 1,248
    // Division by 64 after multiplying by 64 used to maintain integers during calculation.
    std::uint32_t const divisor_x64 =
      (uart_clock_hz * 64U + (oversampling * p_settings.baud_rate) / 2U) /
      (oversampling * p_settings.baud_rate);
    std::uint32_t const integer_divisor = divisor_x64 / 64U;
    std::uint32_t const fractional_divisor = divisor_x64 % 64U;

    // If Divisor Value out of range or 0, return false as a fail-safe.
    if (integer_divisor == 0U ||
        integer_divisor > std::numeric_limits<std::uint16_t>::max()) {
      return false;
    } // Numeric Limits has limits of a numeric type.

    std::uint32_t parity = 0U; // Chooses parity bit check to catch errors when transmission is unexpected.
    switch (p_settings.parity) {
      case settings::parity::none:
        break; // No parity bit transmitted
      case settings::parity::odd:
        parity = LCRH_PEN;
        break; // Parity with odd number of 1 bits
      case settings::parity::even:
        parity = LCRH_PEN | LCRH_EPS;
        break; // Parity with even number of 1 bits
      case settings::parity::forced1:
        parity = LCRH_PEN | LCRH_SPS;
        break; // Parity bit always 1
      case settings::parity::forced0:
        parity = LCRH_PEN | LCRH_SPS | LCRH_EPS;
        break; // Parity bit always 0
    }

    // The LaunchPad diagram routes this UART through PA8 (TX) and PA9 (RX).
    // J14 must be installed in position 2-3 so the header RX signal selects
    // PA9 rather than PB23. UART1 pincm: Pg. 11
    *reg(IOMUX_PA8) = PINCM_PF_UART1 | PINCM_PC_CONNECTED;
    *reg(IOMUX_PA9) = PINCM_PF_UART1 | PINCM_PC_CONNECTED |
                      PINCM_INENA_ENABLE | PINCM_PIPU_ENABLE;

    // Register-level UART1 configuration. Pgs. 1265-1266
    *uart_reg(UART1_RSTCTL) = RSTCTL_KEY | RSTCTL_STICKY_CLEAR |
                              RSTCTL_ASSERT; // Resets UART1 to start configuring system
    *uart_reg(UART1_PWREN) = PWREN_KEY | PWREN_ENABLE; // Enables power, is after reset to
                                                       // be sure power is enabled
    *uart_reg(UART1_CLKSEL) = CLKSEL_BUSCLK;          // Standard bus clock selected.
    *uart_reg(UART1_CLKDIV) = 0U;                     // No clock divider necessary,
                                                      // Fully handled by divisor x64 to
                                                      // properly configure Baud Rate.

    *uart_reg(UART1_CTL0) = CTL0_TXE | CTL0_RXE | CTL0_FEN; // Enables TX/RX abilites and
                                                            // FIFO data transfer
    *uart_reg(UART1_LCRH) = LCRH_WLEN8 | parity |
                  (p_settings.stop == settings::stop_bits::two
                     ? LCRH_STP2
                     : 0U);         // Line control register: 
                                    // 1 Byte(8 bit) transfers, parity chosen,
                                    // default 1 stop bit.
    *uart_reg(UART1_IBRD) = integer_divisor; // For Baud Rate Calculation, integer divider used
                                             // to attain 115,200.
    *uart_reg(UART1_FBRD) = fractional_divisor; // For Baud Rate Calculation, fractional divider
                                                // used to attain 115,200.

    // The UART requires an LCRH(Line control register) write after changing IBRD/FBRD.  Rewriting
    // the complete line-control value also makes the ordering explicit. This is because
    // the divisors are not yet loaded to the Baud Rate Generator, and will be after being called.
    *uart_reg(UART1_LCRH) = LCRH_WLEN8 | parity |
                  (p_settings.stop == settings::stop_bits::two
                     ? LCRH_STP2
                     : 0U);
    *uart_reg(UART1_CTL0) |= CTL0_ENABLE; // Enables UART control register 0. It takes the
                                          // Previous TX/RX and FIFO enabling and adds the enable
                                          // Bit for the UART control.

    *uart_reg(UART1_IFLS) = (*uart_reg(UART1_IFLS) & ~IFLS_RX_MASK) |
                            IFLS_RX_LEVEL_1; // Interrupt FIFO Level-Select
    *uart_reg(UART1_IMASK) = INT_RX | INT_FRMERR | INT_PARERR |
                             INT_BRKERR | INT_OVRERR; // Errors enabled for framing, parity,
                                                      // breaks, and overrun.
    *reg(NVIC_ISER0) = (1U << 13U); // Used to enable NVIC Interrupt.
    return true;
  }

  void driver_write(std::span<std::uint8_t const> p_data) override
  {
    for (std::uint8_t const byte : p_data) {
      while ((*uart_reg(UART1_STAT) & STAT_TXFF) != 0U) {
      } // Sends bits to byte as long as uart still sends data and transmission limit is not reached yet.
      *uart_reg(UART1_TXDATA) = byte; // Sets value of data being transferred to byte
    }
  }

  std::span<std::uint8_t const> driver_receive_buffer() override
  {
    return std::span<std::uint8_t const>(
      receive_storage.data(), receive_storage.size());
  } // Returns the received storage information as a span.

  std::size_t driver_receive_cursor() override
  {
    return receive_write_cursor;
  } // Returns the current cursor
};

extern "C" void UART1_IRQHandler()
{
  uart_serial_driver::handle_interrupt();
}
// Constructs driver here.
static lab6::serial::settings const uart_settings{};  // 115200 baud, 8N1
static uart_serial_driver uart_driver(uart_settings);

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
  while (true) {
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

constexpr device_role this_device = device_role::sensor;
}  // namespace

int main()
{
  std::printf("Hello, World\n");
  // If statement checks if the UART driver is configured, failing should happen at
  // the constructor, not the transfer function.
  if (!uart_driver.is_configured()) {
    std::printf("UART configuration failed\n");
  } else {
    std::printf("UART configured successfully\n");

    static constexpr std::uint8_t uart_tx_test[] =
      "UART TX test: direct driver transmission works\r\n";
    uart_driver.write(std::span<std::uint8_t const>(
      uart_tx_test, sizeof(uart_tx_test) - 1U));

    constexpr std::size_t expected_bytes = sizeof(uart_tx_test) - 1U;
    volatile std::uint32_t timeout = 1'000'000U;
    while (uart_driver.receive_cursor() < expected_bytes && timeout-- != 0U) {
    }

    auto const uart_debug = uart_serial_driver::debug_state();
    std::printf(
      "UART STAT=0x%08lX IIDX=0x%08lX cursor=%lu\n",
      static_cast<unsigned long>(uart_debug.stat),
      static_cast<unsigned long>(uart_debug.iidx),
      static_cast<unsigned long>(uart_driver.receive_cursor()));
    std::printf(
      "UART CTL0=0x%08lX CLKSEL=0x%08lX PWREN=0x%08lX\n",
      static_cast<unsigned long>(uart_debug.ctl0),
      static_cast<unsigned long>(uart_debug.clksel),
      static_cast<unsigned long>(uart_debug.pwren));

    std::span<std::uint8_t const> const received =
      uart_driver.receive_buffer();
    std::printf("Received data: ");
    for (std::size_t i = 0; i < uart_driver.receive_cursor(); ++i) {
      std::printf("%c", received[i]);
    }
    std::printf("\n");
  }

  std::printf("UART done \n");
  if constexpr (this_device == device_role::sensor) {
    // TODO(lab, role 4): Before the scheduler starts:
    //   1. Create sample_request and batch_queue.
    //   2. Create request_task, sampler_task, and transmit_task with
    //      xTaskCreateStatic().
    // request_task and transmit_task share one serial_driver. main()'s stack
    // is reused once the scheduler starts, so don't make it a local here.
  } else {
    xTaskCreateStatic(display_task,
                      "display_task",
                      task_stack_words,
                      nullptr,
                      tskIDLE_PRIORITY + 1,
                      display_task_stack,
                      &display_task_tcb);
  }

  vTaskStartScheduler();

  // Only reached if vTaskStartScheduler() itself fails to start. The
  // idle/timer task memory callbacks required by
  // configSUPPORT_STATIC_ALLOCATION are already supplied by the freertos_kernel
  // project's dpl/StaticAllocs_freertos.c - don't redefine them here.
  while (true) {
  }
}
