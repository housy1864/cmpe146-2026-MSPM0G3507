#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../hal/gpio.hpp"
#include "../hal/spi.hpp"

namespace {

// MSPM0G3507 memory map and register offsets.
// These values replace the device-header register definitions.
namespace mspm0g {

// IOMUX base address. PINCM entries used for pin multiplexing begin here;
// each entry occupies one 32-bit register (4 bytes).
constexpr std::uintptr_t iomux_base = 0x4042'8000u;
// GPIOB base address. This GPIO port carries the SPI1 pins and chip select.
constexpr std::uintptr_t gpiob_base = 0x400A'2000u;
// SPI1 base address. This is the SPI controller's register block.
constexpr std::uintptr_t spi1_base = 0x4046'A000u;

// GPIOB + 0x0800: power-enable register; enables the GPIOB peripheral.
constexpr std::uintptr_t gpiob_pwren = gpiob_base + 0x0800u;
// GPIOB + 0x0804: reset-control register; clears/asserts GPIOB reset.
constexpr std::uintptr_t gpiob_rstctl = gpiob_base + 0x0804u;
// GPIOB + 0x1280: output-data register; reads the current PB output levels.
constexpr std::uintptr_t gpiob_dout = gpiob_base + 0x1280u;
// GPIOB + 0x1290: output-set register; writing 1 drives a PB pin high.
constexpr std::uintptr_t gpiob_doutset = gpiob_base + 0x1290u;
// GPIOB + 0x12A0: output-clear register; writing 1 drives a PB pin low.
constexpr std::uintptr_t gpiob_doutclr = gpiob_base + 0x12A0u;
// GPIOB + 0x12D0: output-enable set register; writing 1 enables PB output.
constexpr std::uintptr_t gpiob_doeset = gpiob_base + 0x12D0u;

// SPI1 + 0x0800: power-enable register; enables SPI1.
constexpr std::uintptr_t spi1_pwren = spi1_base + 0x0800u;
// SPI1 + 0x0804: reset-control register; clears/asserts SPI1 reset.
constexpr std::uintptr_t spi1_rstctl = spi1_base + 0x0804u;
// SPI1 + 0x0814: reset/status register; reports SPI1 reset state.
constexpr std::uintptr_t spi1_gprcm_stat = spi1_base + 0x0814u;
// SPI1 + 0x1000: peripheral clock-divider register.
constexpr std::uintptr_t spi1_clkdiv = spi1_base + 0x1000u;
// SPI1 + 0x1004: clock-source selector; this program selects SYSCLK.
constexpr std::uintptr_t spi1_clksel = spi1_base + 0x1004u;
// SPI1 + 0x1100: control 0; selects frame size, polarity, phase, and format.
constexpr std::uintptr_t spi1_ctl0 = spi1_base + 0x1100u;
// SPI1 + 0x1104: control 1; enables SPI1 and selects controller/MSB operation.
constexpr std::uintptr_t spi1_ctl1 = spi1_base + 0x1104u;
// SPI1 + 0x1108: serial-clock control; SCR sets the SCLK divider.
constexpr std::uintptr_t spi1_clkctl = spi1_base + 0x1108u;
// SPI1 + 0x1110: status register; reports TX/RX FIFO state.
constexpr std::uintptr_t spi1_stat = spi1_base + 0x1110u;
// SPI1 + 0x1130: receive-data register; reading removes one RX FIFO byte.
constexpr std::uintptr_t spi1_rxdata = spi1_base + 0x1130u;
// SPI1 + 0x1140: transmit-data register; writing queues one TX FIFO byte.
constexpr std::uintptr_t spi1_txdata = spi1_base + 0x1140u;

// Power/reset key and control values written to the GPRCM registers.
constexpr std::uint32_t power_key = 0x2600'0000u;       // Unlock power writes.
constexpr std::uint32_t power_enable = 0x0000'0001u;    // Enable peripheral power.
constexpr std::uint32_t reset_key = 0xB100'0000u;       // Unlock reset writes.
constexpr std::uint32_t reset_sticky_clear = 0x0000'0002u; // Clear latched reset.
constexpr std::uint32_t reset_assert = 0x0000'0001u;    // Assert peripheral reset.

// IOMUX pin-control values. PF selects the peripheral function for each pin.
constexpr std::uint32_t iomux_connected = 0x0000'0080u; // Connect pin to function.
constexpr std::uint32_t iomux_input_enable = 0x0004'0000u; // Enable digital input; needed for MISO.
constexpr std::uint32_t iomux_pf_gpio_b17 = 0x0000'0001u; // PB17 as GPIO chip select.
constexpr std::uint32_t iomux_pf_spi1_poci = 0x0000'0003u; // PB7 as SPI1 POCI/MISO.
constexpr std::uint32_t iomux_pf_spi1_pico = 0x0000'0003u; // PB8 as SPI1 PICO/MOSI.
constexpr std::uint32_t iomux_pf_spi1_sclk = 0x0000'0003u; // PB9 as SPI1 SCLK.

// SPI control and status values used by the raw register driver.
constexpr std::uint32_t spi_ctl0_dss_8 = 0x0000'0007u; // CTL0 DSS: 8-bit frames.
constexpr std::uint32_t spi_ctl0_motorola_4wire = 0x0000'0020u; // CTL0: Motorola 4-wire format.
constexpr std::uint32_t spi_ctl0_spo_high = 0x0000'0100u; // CTL0 SPO: SCLK idles high.
constexpr std::uint32_t spi_ctl0_sph_second = 0x0000'0200u; // CTL0 SPH: sample on second edge.
constexpr std::uint32_t spi_ctl1_enable_mask = 0x0000'0001u; // CTL1 enable-bit mask.
constexpr std::uint32_t spi_ctl1_enable = 0x0000'0001u; // CTL1: enable SPI1.
constexpr std::uint32_t spi_ctl1_controller = 0x0000'0004u; // CTL1: controller/master mode.
constexpr std::uint32_t spi_ctl1_msb_first = 0x0000'0010u; // CTL1: transmit MSB first.
constexpr std::uint32_t spi_clksel_sysclk = 0x0000'0008u; // CLKSEL: select SYSCLK.
constexpr std::uint32_t spi_clkctl_scr_mask = 0x0000'03FFu; // CLKCTL SCR: 10-bit divider field.
constexpr std::uint32_t spi_stat_tnf_mask = 0x0000'0002u; // STAT TNF: TX FIFO status bit.
constexpr std::uint32_t spi_stat_tnf_full = 0x0000'0000u; // TNF=0 means TX FIFO full.
constexpr std::uint32_t spi_stat_rfe_mask = 0x0000'0004u; // STAT RFE: RX FIFO status bit.
constexpr std::uint32_t spi_stat_rfe_empty = 0x0000'0004u; // RFE=1 means RX FIFO empty.

constexpr std::uintptr_t iomux_pincm(std::uint32_t p_pin_control_number)
{
  // PINCM names begin at register array index one.
  return iomux_base + (4u * p_pin_control_number);
  //calculates the memory address of a specific IOMUX PINCM register. Each PINCM register occupies 4 bytes
}

// Converts hardware address into volatile register reference.
volatile std::uint32_t& mmio(std::uintptr_t p_address)
{
  return *reinterpret_cast<volatile std::uint32_t*>(p_address);
}

}  // namespace mspm0g

constexpr std::uint32_t bus_clock_hz = 32'000'000u;
// W25Q command opcodes used to control the Winbond flash peripheral.
constexpr std::uint8_t w25q_read_jedec_id = 0x9F;   // Manufacturer ID, represents who made the chip, 0x9Fh. Page 45.
constexpr std::uint8_t w25q_read_data = 0x03;       // Microcontroller reads data starting at a 24-bit address, 0x03h. Page 28
constexpr std::uint8_t w25q_write_enable = 0x06;    // Set the flash write-enable latch. Used before modifying flash memory. Page 24
constexpr std::uint8_t w25q_page_program = 0x02;    // Program data starting at a 24-bit address. Programs the flash chip. Page 35.
constexpr std::uint8_t w25q_read_status_1 = 0x05;   // Read status register 1, including the busy bit. Page 25
constexpr std::uint8_t w25q_sector_erase_4k = 0x20; // Erase one 4 KiB sector at a 24-bit address. Page 37.
constexpr std::uint32_t first_block_address = 0;    // Address of the first flash block.
constexpr std::size_t block_size = 32;              // Maximum number of bytes read or written.
constexpr std::uint32_t peripheral_power_wait_cycles = 8; // How many times for iteration after the peripheral is turned on.
constexpr std::uint32_t spi_poll_timeout = 1'000'000;

std::uint32_t pin_mask(std::uint8_t pin) // Sets all bits valued at 1 to high.
{
  return 1u << pin;
}

}  // namespace

/**
 * @brief GPIO output implementation used for the flash chip-select line.
 *
 * CS is deliberately a normal GPIO rather than the SPI peripheral's automatic
 * chip select. This lets a flash command and its data remain one transaction.
 */
class mspm0g_outpin : public lab1::output_pin // A newer output pin implementation
{
public:
  mspm0g_outpin(std::uint8_t p_port, std::uint8_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
    // The lab uses PB17, which is BoosterPack pin 18.
    mspm0g::mmio(mspm0g::gpiob_pwren) =
        mspm0g::power_key | mspm0g::power_enable;
    for (volatile std::uint32_t i = 0;
         i < peripheral_power_wait_cycles;
         ++i) {
      (void)mspm0g::mmio(mspm0g::gpiob_pwren);
    }
    mspm0g::mmio(mspm0g::gpiob_rstctl) =
        mspm0g::reset_key | mspm0g::reset_sticky_clear |
        mspm0g::reset_assert;
    mspm0g::mmio(mspm0g::iomux_pincm(43)) =
        mspm0g::iomux_connected | mspm0g::iomux_pf_gpio_b17; // Chip Select connected to PB17 pin.
    mspm0g::mmio(mspm0g::gpiob_doutset) = pin_mask(m_pin);
    mspm0g::mmio(mspm0g::gpiob_doeset) = pin_mask(m_pin);
  }

private:
  std::uint8_t m_port;
  std::uint8_t m_pin;

  bool driver_configure(settings const& p_settings) override
  {
    (void)p_settings;
    return m_port == 1 && m_pin == 17;
  }
  // Sets the output to high or low. Turns pin on or off
  void driver_level(bool p_high) override
  {
    if (p_high) {
      mspm0g::mmio(mspm0g::gpiob_doutset) = pin_mask(m_pin);
    }
    else {
      mspm0g::mmio(mspm0g::gpiob_doutclr) = pin_mask(m_pin);
    }
  }

  bool driver_level() override
  {
    return (mspm0g::mmio(mspm0g::gpiob_dout) & pin_mask(m_pin)) != 0;
    // reads the current output state of a GPIO pin -> CS
  }
};

/**
 * @brief Bare-metal SPI1 controller implementing the lab3::spi interface.
 *
 * SPI1 is configured for Motorola mode 0, 8-bit, MSB-first transfers. The
 * peripheral is used only for clocking bytes; chip-select is controlled by
 * the flash_memory class.
 */
class mspm0g_spi : public lab3::spi
{
public:
  mspm0g_spi()
  {
    mspm0g::mmio(mspm0g::gpiob_pwren) =
        mspm0g::power_key | mspm0g::power_enable; // Turns GPIOB peripheral on. Page 672
    mspm0g::mmio(mspm0g::spi1_rstctl) =
        mspm0g::reset_key | mspm0g::reset_sticky_clear |
        mspm0g::reset_assert; // Resets the SPI pin. Pages 673 
    mspm0g::mmio(mspm0g::spi1_pwren) =
        mspm0g::power_key | mspm0g::power_enable; // Enables SPI peripheral pin control, attached to the GPIOB pin. Page 1433
    for (volatile std::uint32_t i = 0;
         i < peripheral_power_wait_cycles;
         ++i) {
      (void)mspm0g::mmio(mspm0g::spi1_pwren);
    }

    /*
      Sets up SPI with pins so it can be used. Does not have its own dedicated pins, so
      SPI requires IOMUX pins to be used.
    */
    // 1. SPI signaling
    mspm0g::mmio(mspm0g::iomux_pincm(26)) =
        mspm0g::iomux_connected | mspm0g::iomux_pf_spi1_sclk; // PB9
    // Assigns SPI Clock to pin 26.
    mspm0g::mmio(mspm0g::iomux_pincm(25)) =
        mspm0g::iomux_connected | mspm0g::iomux_pf_spi1_pico; // PB8
    // Assigns peripheral in(SPI input)/controller out to pin 25, is for the output. PICO
    mspm0g::mmio(mspm0g::iomux_pincm(24)) =
        mspm0g::iomux_connected | mspm0g::iomux_input_enable |
        mspm0g::iomux_pf_spi1_poci; // PB7
    // Assigns SPI output peripheral out(SPI output)/controller in to pin 24. POCI

    configure({});
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    if (p_settings.clock_rate == 0) {
      return false;
    }

    mspm0g::mmio(mspm0g::spi1_ctl1) &= ~mspm0g::spi_ctl1_enable_mask;

    const std::uint32_t frame_format = [&] {
      switch (p_settings.bus_mode) { // Selector for SPI modes: 0-3
        /*
          Motorola SPI frame format is the standard SPI protocol, selected with value 32(0x20).
          SPI was created by Motorola, explaining the standard protocol's name.
          SPI_CTL0_SPH_SECOND sets second clock edge for capturing. Sets to sample on fall and shift on rise. Used by modes 1 and 3.
          SPI_CTL0_SPH_HIGH sets Clock Idle to 1, used by modes 2 and 3.
        */
        // Modes 0 and 3 Sample Data on the rising edge and shifts data on the falling edge.
        // Modes 1 and 2 Sample Data on the falling edge and shifts data on the rising edge.
        // Modes 0 and 2 shift data when chip select is available.
        case mode::m1: return mspm0g::spi_ctl0_motorola_4wire |
                              mspm0g::spi_ctl0_sph_second;
        case mode::m2: return mspm0g::spi_ctl0_motorola_4wire |
                              mspm0g::spi_ctl0_spo_high;
        case mode::m3: return mspm0g::spi_ctl0_motorola_4wire |
                              mspm0g::spi_ctl0_spo_high |
                              mspm0g::spi_ctl0_sph_second;
        case mode::m0: return mspm0g::spi_ctl0_motorola_4wire;
      } // Pages 1465, 66. CTL0 is used for the format of sending signals and the size of data.
      return mspm0g::spi_ctl0_motorola_4wire;
    }();

    mspm0g::mmio(mspm0g::spi1_clksel) = mspm0g::spi_clksel_sysclk;
    mspm0g::mmio(mspm0g::spi1_clkdiv) = 0;
    mspm0g::mmio(mspm0g::spi1_ctl0) =
        mspm0g::spi_ctl0_dss_8 | frame_format;
    mspm0g::mmio(mspm0g::spi1_ctl1) = //allows us to modify it and use the MSB bit first
        mspm0g::spi_ctl1_controller | mspm0g::spi_ctl1_msb_first;

    // fSCLK = fBUSCLK / (2 * (SCR + 1)). Chapter 26 SPI clock control 26.2.2.4 Clock Generation
    // maximum clock rate is never exceeded.
    std::uint32_t divider = bus_clock_hz / (2u * p_settings.clock_rate);
    if (divider == 0) {
      divider = 1;
    }
    --divider;
    if (divider > 1023u) {
      divider = 1023u;
    } // P1469 - SCR is a value from 0-1023
    mspm0g::mmio(mspm0g::spi1_clkctl) =
        divider & mspm0g::spi_clkctl_scr_mask;
    mspm0g::mmio(mspm0g::spi1_ctl1) |= mspm0g::spi_ctl1_enable;
    return true;
  }

  // Used to move bytes through the MSPM0 board's SPI peripheral
  void driver_transfer(std::span<std::uint8_t const> p_data_out,
                       std::span<std::uint8_t> p_data_in,
                       std::uint8_t p_filler) override
  {
    std::uint32_t timeout = spi_poll_timeout;
    /*  
        Clears data from older transfers to allow new command.
        Timeout added to remove timeout issues with clearing data.
    */
    while ((mspm0g::mmio(mspm0g::spi1_stat) &
            mspm0g::spi_stat_rfe_mask) != mspm0g::spi_stat_rfe_empty) {
      if (timeout-- == 0) {
        return;
      }
      (void)mspm0g::mmio(mspm0g::spi1_rxdata);
    }

    // Finds the number of bytes to send and receive during transfer
    const std::size_t transfer_size =
        p_data_out.size() > p_data_in.size() ? p_data_out.size()
                                             : p_data_in.size();

    for (std::size_t i = 0; i < transfer_size; ++i) {
      //tx is the byte being selected for transfer
      const std::uint8_t tx =
          i < p_data_out.size() ? p_data_out[i] : p_filler;
      timeout = spi_poll_timeout; // Used to avoid timeout
      while ((mspm0g::mmio(mspm0g::spi1_stat) &
              mspm0g::spi_stat_tnf_mask) == mspm0g::spi_stat_tnf_full) {
        if (timeout-- == 0) {
          return;
        }
      }
      mspm0g::mmio(mspm0g::spi1_txdata) = tx;
      timeout = spi_poll_timeout;
      while ((mspm0g::mmio(mspm0g::spi1_stat) &
              mspm0g::spi_stat_rfe_mask) == mspm0g::spi_stat_rfe_empty) {
        if (timeout-- == 0) {
          return;
        }
      }
      /*
        Reads data. RXDATA(0x00001130 offset) removes received byte
        from queue. If caller has input buffer, the byte is stored into flash.
      */
      const std::uint8_t rx = static_cast<std::uint8_t>(
          mspm0g::mmio(mspm0g::spi1_rxdata));
      if (i < p_data_in.size()) {
        p_data_in[i] = rx;
      }
    }
  }
};

/**
 * @brief Generic SPI NOR flash driver for the W25Q command set.
 */
class flash_memory
{
public:
  flash_memory(lab3::spi* p_spi, lab1::output_pin* p_chip_select)
    : m_spi(p_spi)                 // SPI driver selected
    , m_chip_select(p_chip_select) // Selects flash
  {
    m_chip_select->configure( // Uses pin high/low for supply voltage
        {.resistor = lab1::pin_resistor::pull_up, .open_drain = false});
    deselect();
  }

  // Sends JEDEC ID command. JEDEC ID is a standard 3-byte ID value
  // ID Value identifies the flash memory peripheral
  // 2. Reads and prints the manufacturing ID.
  std::array<std::uint8_t, 3> read_jedec_id()
  {
    const std::array<std::uint8_t, 1> command = {
      w25q_read_jedec_id}; // 0xFF is a default value.
    std::array<std::uint8_t, 3> response{};

    std::printf("JEDEC TX: %02X %02X %02X %02X\n",
                command[0], command[1], command[2], command[3]);
    transfer(command, response);
    // cs
    m_spi->transfer(command, {});
    m_spi->transfer({}, response);
    std::printf("JEDEC RX: %02X %02X %02X %02X\n",
                response[0], response[1], response[2], response[3]);

    return {response[0], response[1], response[2]}; // This represents the 3 byte JEDEC ID.
  }
  // 2. Reads and prints the memory capacity
  // Reads JEDEC ID, determines the flash size from it.
  std::uint32_t capacity_bytes()
  {
    const auto id = read_jedec_id();
    // Winbond's density byte uses 0x16 for W25Q32 (32 Mbit), 0x17 for
    // W25Q64, etc. Its numeric value is log2(capacity in bytes).
    if (id[2] > 31) {
      return 0;
    }
    return 1u << id[2];
  }
  // 3. Used for reading data stored in flash
  void read_block(std::span<std::uint8_t> p_data)
  {
    const std::size_t count = p_data.size(); // # of bytes to read
    std::array<std::uint8_t, 4> command{}; // Flash command build
    // Command array represents a flash command. One command byte to read,
    // 3 bytes for an address. Also read as 24-bit(3-Byte).
    // 3. Block Read
    command[0] = w25q_read_data;
    command[1] = static_cast<std::uint8_t>(first_block_address >> 16);  // Address Byte 1
    command[2] = static_cast<std::uint8_t>(first_block_address >> 8);   // Address Byte 2
    command[3] = static_cast<std::uint8_t>(first_block_address);        // Address Byte 3
    transfer(command, p_data); // Microcontroller requests data from SPI, SPI
                                                                  // responds to microcontroller command read with data response.
  }
  // 4. Used for writing data to the flash
  bool write_block(std::span<std::uint8_t const> p_data)
  {
    // Explicit write size for the data to be written to flash, 32 bytes.
    const std::size_t count = p_data.size() > block_size ? block_size
                                                         : p_data.size();
    if (count == 0) {
      return true;
    }

    // A sector erase is needed before programming arbitrary data into NOR
    // flash because programming can only change 1 bits to 0 bits.
    if (!write_enable()) {
      return false;
    }
    std::array<std::uint8_t, 4> erase_command = {
      w25q_sector_erase_4k, 0x00, 0x00, 0x00}; // Erases 4 kib. Clears flash chip data before modifying with new value.
    command(erase_command);
    if (!wait_until_ready()) { // Makes sure no other data or transfer happens before writing data
      return false;
    }

    if (!write_enable()) { // Calls the write_enable command for the NOR flash chip before writing any data
      return false;
    }
    // TO-DO: ELIMINATE ADDITIONAL BLOCK SIZE BUFFERS AND USE SPANS DIRECTLY
    // 4. Block Write
    std::array<std::uint8_t, 4 + block_size> program{};
    program[0] = w25q_page_program; // Starts at a 24bit(3-Byte) address
    program[1] = static_cast<std::uint8_t>(first_block_address >> 16);  // Address Byte
    program[2] = static_cast<std::uint8_t>(first_block_address >> 8);   // Address Byte 3
    program[3] = static_cast<std::uint8_t>(first_block_address);        // Address Byte 3
    transfer(command, p_data); // Microcontroller requests data from SPI, SPI
    // responds to microcontroller command read with data response.
    return wait_until_ready();
  }

private:
  void select() // Selects flash chip
  {
    m_chip_select->level(false);
  }

  void deselect() // Deselects flash chip
  {
    m_chip_select->level(true);
  }

  template<std::size_t N>
  void command(std::array<std::uint8_t, N> const& p_command) // Array contains data
  {
    select();
    m_spi->transfer(p_command, {}, lab3::spi::default_filler);
    deselect();
  }

  void command(std::span<std::uint8_t const> p_command) // Span contains addresses and count of elements, no data stored.
  {
    select();
    m_spi->transfer(p_command, {}, lab3::spi::default_filler);
    deselect();
  }

  template<std::size_t N>
  void transfer(std::array<std::uint8_t, N> const& p_out,
                std::array<std::uint8_t, N>& p_in) // Array
  {
    transfer(std::span<std::uint8_t const>(p_out.data(), N),
             std::span<std::uint8_t>(p_in.data(), N));
  }

  void transfer(std::span<std::uint8_t const> p_out,
                std::span<std::uint8_t> p_in) // Span
  {
    select();
    m_spi->transfer(p_out, p_in, lab3::spi::default_filler);
    deselect();
  }

  bool write_enable() // Enables ability to modify data on NOR flash chip, not actually writing. Writing done by page program
  {
    const std::array<std::uint8_t, 1> command_bytes = {w25q_write_enable};
    command(command_bytes);
    return true;
  }

  bool wait_until_ready() // Checks status of bit sending to make sure no other commands are being executed. Greenlights for write.
  {
    for (std::uint32_t attempt = 0; attempt < 10'000'000u; ++attempt) {
      const std::array<std::uint8_t, 2> command_bytes = {
        w25q_read_status_1, lab3::spi::default_filler};
      std::array<std::uint8_t, 2> response{};
      transfer(command_bytes, response);
      if ((response[1] & 0x01u) == 0) {
        return true;
      }
    }
    return false;
  }

  lab3::spi* m_spi = nullptr;
  lab1::output_pin* m_chip_select = nullptr;
};

int main()
{
  std::printf("Main started\n");
  mspm0g_spi spi;
  mspm0g_outpin chip_select(1, 17);
  flash_memory flash(&spi, &chip_select);

  spi.configure({.clock_rate = 500'000, .bus_mode = lab3::spi::mode::m0});
  std::printf("SPI configured\n");
  const auto id = flash.read_jedec_id(); // Used to print the manufacturer's ID.
  std::printf("Touched\n");
  const std::uint32_t capacity = flash.capacity_bytes(); // Used to print size of flash chip.
  std::array<std::uint8_t, block_size> block{};
  flash.read_block(block);

  std::printf("JEDEC manufacturer ID: 0x%02X\n", id[0]);
  std::printf("JEDEC type: 0x%02X, density code: 0x%02X\n", id[1], id[2]);
  std::printf("Flash capacity: %lu bytes (%lu KiB)\n",
              static_cast<unsigned long>(capacity),
              static_cast<unsigned long>(capacity / 1024u));
  std::printf("First 32-byte block:\n");
  for (std::size_t i = 0; i < block.size(); ++i) {
    std::printf("%02X%c", block[i], (i % 16 == 15) ? '\n' : ' ');
  }

  char input[block_size + 1]{};
  std::printf("Enter up to 32 bytes to write to address 0 (empty line skips):\n");
  if (std::fgets(input, sizeof(input), stdin) != nullptr) {
    std::size_t length = std::strcspn(input, "\r\n");
    if (length != 0) {
      const bool success = flash.write_block(
          std::span<std::uint8_t const>(
              reinterpret_cast<std::uint8_t const*>(input), length));
      std::printf("Flash write %s\n", success ? "complete" : "failed");
    }
  }

  while (true) {
    continue;
  }
}
