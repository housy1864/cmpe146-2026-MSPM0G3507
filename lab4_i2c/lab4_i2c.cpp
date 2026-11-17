
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cmath>

#include "../hal/accelerometer.hpp"
#include "../hal/i2c.hpp"

// ============================================================
// SOFTWARE I2C DRIVER
// Uses PA0 as SDA and PA1 as SCL.
// ============================================================


class software_i2c : public lab4::i2c
{
private:

    // GPIO Port B registers
    static constexpr std::uintptr_t GPIOB_BASE = 0x400A2000;

    static constexpr std::uintptr_t GPIOB_DOUT = GPIOB_BASE + 0x1280;
    static constexpr std::uintptr_t GPIOB_DOE  = GPIOB_BASE + 0x12C0;
    static constexpr std::uintptr_t GPIOB_DIN  = GPIOB_BASE + 0x1380;

    static constexpr std::uintptr_t GPIOB_PWREN =
    GPIOB_BASE + 0x800;

    // PB3 = SDA, PB2 = SCL
    static constexpr std::uint32_t SDA_MASK = (1u << 3);
    static constexpr std::uint32_t SCL_MASK = (1u << 2);

    // IOMUX base = 0x40428000
    // PINCM array begins at offset 0x04.

    static constexpr std::uintptr_t IOMUX_PB2 = 0x40428040;
    static constexpr std::uintptr_t IOMUX_PB3 = 0x40428044;

    // THIS FUNCTION WAS MISSING
    static volatile std::uint32_t* reg(std::uintptr_t address)
    {
        return reinterpret_cast<volatile std::uint32_t*>(address);
    }

    // Configure PB2 and PB3 as GPIO inputs with internal pull-ups
    static bool gpio_init()
    {
        // Enable GPIO Port B
        *reg(GPIOB_PWREN) = 1;

        // Configure PB2 = SCL and PB3 = SDA as GPIO
        // Enable input and internal pull-up
        *reg(IOMUX_PB2) = 0x00060081;
        *reg(IOMUX_PB3) = 0x00060081;

        // Set output latches LOW
        *reg(GPIOB_DOUT) &= ~(SDA_MASK | SCL_MASK);

        // Release both pins (open-drain operation)
        *reg(GPIOB_DOE) &= ~(SDA_MASK | SCL_MASK);

        return true;
    }

    // Small delay between I2C transitions
    static void i2c_delay()
    {
        for (volatile std::uint32_t i = 0; i < 80; ++i)
        {
        }
    }

    // Pull SDA LOW
    void sda_low()
    {
        *reg(GPIOB_DOUT) &= ~SDA_MASK;
        *reg(GPIOB_DOE) |= SDA_MASK;
    }

    // Release SDA
    void sda_release()
    {
        *reg(GPIOB_DOE) &= ~SDA_MASK;
    }

    // Pull SCL LOW
    void scl_low()
    {
        *reg(GPIOB_DOUT) &= ~SCL_MASK;
        *reg(GPIOB_DOE) |= SCL_MASK;
    }

    // Release SCL
    void scl_release()
    {
        *reg(GPIOB_DOE) &= ~SCL_MASK;
    }

    // Read SDA
    bool sda_read()
    {
        return ((*reg(GPIOB_DIN) & SDA_MASK) != 0);
    }

    // Read SCL
    bool scl_read()
    {
        return ((*reg(GPIOB_DIN) & SCL_MASK) != 0);
    }

    // Generate START condition
    void start_condition()
    {
        sda_release();
        scl_release();
        i2c_delay();

        sda_low();
        i2c_delay();

        scl_low();
        i2c_delay();
    }

    // Generate STOP condition
    void stop_condition()
    {
        sda_low();
        i2c_delay();

        scl_release();
        i2c_delay();

        sda_release();
        i2c_delay();
    }

    // Generate repeated START condition
    void repeated_start()
    {
        sda_release();
        i2c_delay();

        scl_release();
        i2c_delay();

        sda_low();
        i2c_delay();

        scl_low();
        i2c_delay();
    }

    // Send one bit
    void send_bit(bool bit)
    {
        if (bit)
        {
            sda_release();
        }
        else
        {
            sda_low();
        }

        i2c_delay();

        scl_release();
        i2c_delay();

        scl_low();
        i2c_delay();
    }

    // Receive one bit
    bool receive_bit()
    {
        sda_release();
        i2c_delay();

        scl_release();
        i2c_delay();

        bool bit = sda_read();

        scl_low();
        i2c_delay();

        return bit;
    }

    // Send one byte, MSB first
    // Return true if the device acknowledges
    bool send_byte(std::uint8_t data)
    {
        for (int bit = 7; bit >= 0; --bit)
        {
            send_bit((data & (1u << bit)) != 0);
        }

        // Receive ACK on the ninth clock
        return !receive_bit();
    }

    // Receive one byte
    // send_ack = true means ACK
    // send_ack = false means NACK
    std::uint8_t receive_byte(bool send_ack)
    {
        std::uint8_t data = 0;

        for (int bit = 7; bit >= 0; --bit)
        {
            if (receive_bit())
            {
                data |= (1u << bit);
            }
        }

        // Master sends ACK or NACK
        send_bit(!send_ack);

        return data;
    }

    // Configure software I2C
    bool driver_configure(settings const& p_settings) override
    {
        (void)p_settings;

        // Initialize GPIO BEFORE accessing either pin
        if (!gpio_init())
        {
            return false;
        }

        // Release both I2C lines
        sda_release();
        scl_release();

        i2c_delay();

        // Verify idle bus state
        bool sda = sda_read();
        bool scl = scl_read();

        std::printf("SDA = %d, SCL = %d\n", sda, scl);
        std::fflush(stdout);

        return sda && scl;
    }

    // Perform an I2C transaction
    bool driver_transaction(
        std::uint8_t address,
        std::span<std::uint8_t const> data_out,
        std::span<std::uint8_t> data_in) override
    {
        if (data_out.empty() && data_in.empty())
        {
            return false;
        }

        start_condition();

        // Send address with write bit or read bit
        std::uint8_t address_byte =
            static_cast<std::uint8_t>(
                (address << 1) | (data_out.empty() ? 1 : 0));

        if (!send_byte(address_byte))
        {
            stop_condition();
            return false;
        }

        // Send outgoing data
        for (std::size_t i = 0; i < data_out.size(); ++i)
        {
            if (!send_byte(data_out[i]))
            {
                stop_condition();
                return false;
            }
        }

        // If reading is also requested, use repeated START
        if (!data_in.empty())
        {
            if (!data_out.empty())
            {
                repeated_start();

                // Send address with READ bit
                if (!send_byte(
                    static_cast<std::uint8_t>((address << 1) | 1)))
                {
                    stop_condition();
                    return false;
                }
            }

            // Read incoming bytes
            for (std::size_t i = 0; i < data_in.size(); ++i)
            {
                bool send_ack = (i + 1 < data_in.size());
                data_in[i] = receive_byte(send_ack);
            }
        }

        stop_condition();

        return true;
    }
};


// ============================================================
// BMI160 ACCELEROMETER DRIVER
// ============================================================

class bmi160 : public lab4::accelerometer
{
private:

    static constexpr std::uint8_t BMI160_ADDRESS = 0x69;

    static constexpr float SENSITIVITY = 16384.0f;

    lab4::i2c& i2c_bus;

    bool last_read_success = false;

    // Write one byte to a BMI160 register
    bool write_register(std::uint8_t address, std::uint8_t value)
    {
        std::uint8_t data[2] = {address, value};

        return i2c_bus.transaction(
            BMI160_ADDRESS,
            std::span<std::uint8_t const>(data, 2),
            std::span<std::uint8_t>()
        );
    }

    // Read multiple bytes starting from a register
    bool read_registers(
        std::uint8_t address,
        std::uint8_t* buffer,
        std::size_t length)
    {
        return i2c_bus.transaction(
            BMI160_ADDRESS,
            std::span<std::uint8_t const>(&address, 1),
            std::span<std::uint8_t>(buffer, length)
        );
    }

    // Read acceleration data
    read_t driver_read() override
    {
        std::uint8_t data[6] = {};

        // Acceleration registers start at 0x12
        if (!read_registers(0x12, data, 6))
        {
            last_read_success = false;
            return {0.0f, 0.0f, 0.0f};
        }

        last_read_success = true;

        // Combine low and high bytes into signed 16-bit values
        std::int16_t raw_x =
            static_cast<std::int16_t>(
                (static_cast<std::uint16_t>(data[1]) << 8) | data[0]);

        std::int16_t raw_y =
            static_cast<std::int16_t>(
                (static_cast<std::uint16_t>(data[3]) << 8) | data[2]);

        std::int16_t raw_z =
            static_cast<std::int16_t>(
                (static_cast<std::uint16_t>(data[5]) << 8) | data[4]);

        read_t result;

        result.x = static_cast<float>(raw_x) / SENSITIVITY;
        result.y = static_cast<float>(raw_y) / SENSITIVITY;
        result.z = static_cast<float>(raw_z) / SENSITIVITY;

        return result;
    }

public:

    // Connect BMI160 to the I2C bus
    explicit bmi160(lab4::i2c& bus)
        : i2c_bus(bus)
    {
    }

    // Initialize the BMI160
    bool initialize()
    {
        // First verify the chip ID
        std::uint8_t chip_id = 0;

        if (!read_registers(0x00, &chip_id, 1))
        {
            return false;
        }

        if (chip_id != 0xD1)
        {
            return false;
        }

        // Command register 0x7E
        // Command 0x11 switches accelerometer to normal mode
        if (!write_register(0x7E, 0x11))
        {
            return false;
        }

        // Wait for accelerometer startup
        for (volatile std::uint32_t i = 0; i < 500000; ++i)
        {
        }

        return true;
    }

    // Check whether the last sensor read succeeded
    bool read_ok() const
    {
        return last_read_success;
    }
};

int main()
{
    std::printf("\nBMI160 I2C Lab 4\n");
    software_i2c i2c;

    if (!i2c.configure({100000}))
    {
        std::printf("ERROR: I2C GPIO initialization failed.\n");
        std::fflush(stdout);
        while (true) {}
    }

    bmi160 imu(i2c);

    if (!imu.initialize())
    {
        std::printf("ERROR: BMI160 not detected. Check wiring and address.\n");
        std::fflush(stdout);
        while (true) {}
    }

    std::printf("BMI160 detected successfully!\n");
    std::printf("Reading accelerometer data...\n\n");
    std::fflush(stdout);

    constexpr float PI = 3.14159265f;

    while (true)
    {
        lab4::accelerometer::read_t acceleration = imu.read();

        if (!imu.read_ok())
        {
            std::printf("ERROR: Failed to read accelerometer.\n");
        }
        else
        {
            float roll =
                std::atan2(acceleration.y, acceleration.z)
                * 180.0f / PI;

            float pitch =
                std::atan2(
                    -acceleration.x,
                    std::sqrt(
                        acceleration.y * acceleration.y +
                        acceleration.z * acceleration.z
                    )
                ) * 180.0f / PI;

            std::printf(
                "Acceleration X: %7.3f g | "
                "Y: %7.3f g | "
                "Z: %7.3f g | "
                "Roll: %7.2f deg | "
                "Pitch: %7.2f deg\n",
                acceleration.x,
                acceleration.y,
                acceleration.z,
                roll,
                pitch
            );
        }

        std::fflush(stdout);

        for (volatile std::uint32_t i = 0; i < 2000000; ++i)
        {
        }
    }
}
