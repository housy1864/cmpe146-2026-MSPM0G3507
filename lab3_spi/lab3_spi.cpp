#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <ti/devices/msp/m0p/mspm0g350x.h> // Lists the addresses of hardware

#include "../hal/gpio.hpp"
#include "../hal/spi.hpp"

namespace {

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
    GPIOB->GPRCM.PWREN = GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE;
    for (volatile std::uint32_t i = 0;
         i < peripheral_power_wait_cycles;
         ++i) {
      (void)GPIOB->GPRCM.PWREN;
    }
    GPIOB->GPRCM.RSTCTL = GPIO_RSTCTL_KEY_UNLOCK_W |
                          GPIO_RSTCTL_RESETSTKYCLR_CLR |
                          GPIO_RSTCTL_RESETASSERT_ASSERT;
    IOMUX->SECCFG.PINCM[43] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM43_PF_GPIOB_DIO17; // The output pin for SPI is GPIOB 17.
    GPIOB->DOUTSET31_0 = pin_mask(m_pin); // Set to the output pin being used.
    GPIOB->DOESET31_0 = pin_mask(m_pin);  // Enables the GPIO output pin
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
      GPIOB->DOUTSET31_0 = pin_mask(m_pin);
    }
    else {
      GPIOB->DOUTCLR31_0 = pin_mask(m_pin);
    }
  }

  bool driver_level() override
  {
    return (GPIOB->DOUT31_0 & pin_mask(m_pin)) != 0; // Sets output pin as output for GPIOB port.
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
    GPIOB->GPRCM.PWREN = GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE; // GPIOB enabled
    SPI1->GPRCM.PWREN = SPI_PWREN_KEY_UNLOCK_W | SPI_PWREN_ENABLE_ENABLE; // Enables power to peripheral
    for (volatile std::uint32_t i = 0;
         i < peripheral_power_wait_cycles;
         ++i) {
      (void)SPI1->GPRCM.PWREN;
    }
   // Reset control for internal SPI module
    SPI1->GPRCM.RSTCTL = SPI_RSTCTL_KEY_UNLOCK_W |
                         SPI_RSTCTL_RESETSTKYCLR_CLR |
                         SPI_RSTCTL_RESETASSERT_ASSERT;

    /*
      Sets up SPI with pins so it can be used. Does not have its own dedicated pins, so
      SPI requires IOMUX pins to be used.
    */
    // 1. SPI signaling
    IOMUX->SECCFG.PINCM[26] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM26_PF_SPI1_SCLK; 
    // Assigns SPI Clock to pin PB9.
    IOMUX->SECCFG.PINCM[25] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM25_PF_SPI1_PICO; 
    // Assigns peripheral in(SPI input)/controller out to pin PB8, is for the output. PICO
    IOMUX->SECCFG.PINCM[24] = IOMUX_PINCM_PC_CONNECTED |
                              IOMUX_PINCM_INENA_ENABLE |
                              IOMUX_PINCM24_PF_SPI1_POCI; 
    // Assigns SPI output peripheral out(SPI output)/controller in to pin PB9. POCI

    configure({});
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    if (p_settings.clock_rate == 0) {
      return false;
    }

    SPI1->CTL1 &= ~SPI_CTL1_ENABLE_MASK; // Enables module

    const std::uint32_t frame_format = [&] {
      switch (p_settings.bus_mode) { // Selector for SPI modes: 0-3
        /*
          Motorola SPI frame format is the standard SPI protocol, selected with value 32(0x20).
          SPI was created by Motorola, explaining the standard protocol's name.
          SPI_CTL0_SPH_SECOND sets second clock edge for capturing. Sets to sample on fall and shift on rise. Used by modes 1 and 3.
          SPI_CTL0_SPH_HIGH sets Clock Idle to 1, used by modes 2 and 3.

        */
        case mode::m1: return SPI_CTL0_FRF_MOTOROLA_4WIRE | SPI_CTL0_SPH_SECOND; // Clock idle(polarity) high. Samples on fall and shifts on rise.
        case mode::m2: return SPI_CTL0_FRF_MOTOROLA_4WIRE | SPI_CTL0_SPO_HIGH; // Clock idle(polarity) low. Samples on rise and shifts on fall. 
        case mode::m3: return SPI_CTL0_FRF_MOTOROLA_4WIRE |
                              SPI_CTL0_SPO_HIGH | SPI_CTL0_SPH_SECOND; // Clock idle(polarity) low. Samples on fall and shifts on rise.
        case mode::m0: return SPI_CTL0_FRF_MOTOROLA_4WIRE; // Samples on rising edge, shifts on falling. Clock Idle is low.
      }
      return std::uint32_t{SPI_CTL0_FRF_MOTOROLA_4WIRE};
    }();

    SPI1->CLKSEL = SPI_CLKSEL_SYSCLK_SEL_ENABLE;            // Chooses sysclk for the SPI clock
    SPI1->CLKDIV = SPI_CLKDIV_RATIO_DIV_BY_1;               // Clock uses a divider, and is divided by 1
    SPI1->CTL0 = SPI_CTL0_DSS_DSS_8 | frame_format;         // Data size select set number of bits per transfer to 8, chooses the current mode of SPI chosen
    SPI1->CTL1 = SPI_CTL1_CP_ENABLE | SPI_CTL1_MSB_ENABLE;  // Controller mode selected and most significant bit first for order of bits in transmission.

    // fSCLK = fBUSCLK / (2 * (SCR + 1)). Round down so the requested
    // maximum clock rate is never exceeded.
    std::uint32_t divider = bus_clock_hz / (2u * p_settings.clock_rate);
    if (divider == 0) {
      divider = 1;
    }
    --divider;
    if (divider > 1023u) {
      divider = 1023u;
    }
    SPI1->CLKCTL = divider & SPI_CLKCTL_SCR_MASK;
    SPI1->CTL1 |= SPI_CTL1_ENABLE_ENABLE;
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
    while ((SPI1->STAT & SPI_STAT_RFE_MASK) != SPI_STAT_RFE_EMPTY) {
      if (timeout-- == 0) {
        return;
      }
      (void)SPI1->RXDATA;
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
      while ((SPI1->STAT & SPI_STAT_TNF_MASK) == SPI_STAT_TNF_FULL) {
        if (timeout-- == 0) {
          return;
        }
      }
      SPI1->TXDATA = tx; // Transmits bits to peripheral
      timeout = spi_poll_timeout;
      while ((SPI1->STAT & SPI_STAT_RFE_MASK) == SPI_STAT_RFE_EMPTY) {
        if (timeout-- == 0) {
          return;
        }
      }
      /* 
        Reads data. RXDATA(0x00001130 offset) removes received byte
        from queue. If caller has input buffer, the byte is stored into flash.
      */
      const std::uint8_t rx = static_cast<std::uint8_t>(SPI1->RXDATA);
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
    const std::array<std::uint8_t, 4> command = {
      w25q_read_jedec_id, 0xFF, 0xFF, 0xFF}; // 0xFF is a default value.
    std::array<std::uint8_t, 4> response{};
  transfer(command, response);

    std::printf("JEDEC TX: %02X %02X %02X %02X\n",
            command[0], command[1], command[2], command[3]);

    std::printf("JEDEC RX: %02X %02X %02X %02X\n",
            response[0], response[1], response[2], response[3]);
    return {response[1], response[2], response[3]}; // This represents the 3 byte JEDEC ID.
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
    std::array<std::uint8_t, 4 + block_size> command{}; // Flash command build
    std::array<std::uint8_t, 4 + block_size> response{}; // Flash responds and sends data
    // Command array represents a flash command. One command byte to read,
    // 3 bytes for an address. Also read as 24 bits.
    // 3. Block Read
    command[0] = w25q_read_data;
    command[1] = static_cast<std::uint8_t>(first_block_address >> 16);  // Address Byte
    command[2] = static_cast<std::uint8_t>(first_block_address >> 8);   // Address Byte
    command[3] = static_cast<std::uint8_t>(first_block_address);        // Address Byte
    transfer(std::span<std::uint8_t const>(command.data(), 4 + count),
             std::span<std::uint8_t>(response.data(), 4 + count)); // Microcontroller requests data from SPI, SPI 
                                                                  // responds to microcontroller command read with data response.
    std::memcpy(p_data.data(), response.data() + 4, count); // Copies from SPI buffer to controller buffer
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
    // 4. Block Write
    std::array<std::uint8_t, 4 + block_size> program{};
    program[0] = w25q_page_program; // Starts at a 24 bit address
    program[1] = static_cast<std::uint8_t>(first_block_address >> 16);  // Address Byte 1
    program[2] = static_cast<std::uint8_t>(first_block_address >> 8);   // Address Byte 2
    program[3] = static_cast<std::uint8_t>(first_block_address);        // Address Byte 3
    std::memcpy(program.data() + 4, p_data.data(), count);// Start from page program, copies 
                                                          // bytes from microcontroller to flash chip
    command(std::span<std::uint8_t const>(program.data(), 4 + count));
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

  bool write_enable()
  {
    const std::array<std::uint8_t, 1> command_bytes = {w25q_write_enable};
    command(command_bytes);
    return true;
  }

  bool wait_until_ready()
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

  std::printf("JEDEC manufacturer ID: 0x%02X\n", id[0]); // Step 2
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
