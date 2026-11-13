
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

    // GPIO register addresses
    static constexpr std::uintptr_t GPIOA_DOUT = 0x400A1280;
    static constexpr std::uintptr_t GPIOA_DIN  = 0x400A1380;
    static constexpr std::uintptr_t GPIOA_DOE  = 0x400A12C0;

    // Pin control register addresses
    static constexpr std::uintptr_t IOMUX_PA0 = 0x40428000;
    static constexpr std::uintptr_t IOMUX_PA1 = 0x40428004;

    // PA0 = SDA, PA1 = SCL
    static constexpr std::uint32_t SDA_MASK = (1u << 0);
    static constexpr std::uint32_t SCL_MASK = (1u << 1);

    // Access a memory-mapped hardware register
    static volatile std::uint32_t* reg(std::uintptr_t address)
    {
        return reinterpret_cast<volatile std::uint32_t*>(address);
    }

    // Small delay between I2C transitions
    static void i2c_delay()
    {
        for (volatile std::uint32_t i = 0; i < 80; ++i)
        {
        }
    }

    // Configure PA0 and PA1 as GPIO
    void gpio_init()
    {
        *reg(IOMUX_PA0) = 0x00040081;
        *reg(IOMUX_PA1) = 0x00040081;

        // Set output values LOW before enabling outputs
        *reg(GPIOA_DOUT) &= ~(SDA_MASK | SCL_MASK);

        // Release both pins (input/high impedance)
        *reg(GPIOA_DOE) &= ~(SDA_MASK | SCL_MASK);
    }

    // Pull SDA LOW
    void sda_low()
    {
        *reg(GPIOA_DOUT) &= ~SDA_MASK;
        *reg(GPIOA_DOE) |= SDA_MASK;
    }

    // Release SDA
    void sda_release()
    {
        *reg(GPIOA_DOE) &= ~SDA_MASK;
    }

    // Pull SCL LOW
    void scl_low()
    {
        *reg(GPIOA_DOUT) &= ~SCL_MASK;
        *reg(GPIOA_DOE) |= SCL_MASK;
    }

    // Release SCL
    void scl_release()
    {
        *reg(GPIOA_DOE) &= ~SCL_MASK;
    }

    // Read SDA
    bool sda_read()
    {
        return ((*reg(GPIOA_DIN) & SDA_MASK) != 0);
    }

    // Read SCL
    bool scl_read()
    {
        return ((*reg(GPIOA_DIN) & SCL_MASK) != 0);
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

        gpio_init();

        sda_release();
        scl_release();

        i2c_delay();

        // Both lines should be HIGH when idle
        return sda_read() && scl_read();
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


// ============================================================
// MAIN
// ============================================================

int main()
{
    std::printf("\nBMI160 I2C Lab 4\n");

    // Create software I2C driver
    software_i2c i2c;

    // Configure I2C bus
    if (!i2c.configure({100000}))
    {
        std::printf("ERROR: I2C GPIO initialization failed.\n");

        while (true)
        {
        }
    }

    // Create BMI160 object
    bmi160 imu(i2c);

    // Initialize sensor
    if (!imu.initialize())
    {
        std::printf(
            "ERROR: BMI160 not detected. Check wiring and address.\n");

        while (true)
        {
        }
    }

    std::printf("BMI160 detected successfully!\n");

    constexpr float PI = 3.14159265f;

    while (true)
    {
        // Read X, Y, and Z acceleration
        lab4::accelerometer::read_t acceleration = imu.read();

        if (!imu.read_ok())
        {
            std::printf("ERROR: Failed to read accelerometer.\n");
        }
        else
        {
            // Calculate roll
            float roll =
                std::atan2(acceleration.y, acceleration.z)
                * 180.0f / PI;

            // Calculate pitch
            float pitch =
                std::atan2(
                    -acceleration.x,
                    std::sqrt(
                        acceleration.y * acceleration.y +
                        acceleration.z * acceleration.z
                    )
                ) * 180.0f / PI;

            // Print acceleration and orientation
            std::printf(
                "X: %7.3f g | Y: %7.3f g | Z: %7.3f g | "
                "Roll: %7.2f deg | Pitch: %7.2f deg\n",
                acceleration.x,
                acceleration.y,
                acceleration.z,
                roll,
                pitch
            );
        }

        // Delay between measurements
        for (volatile std::uint32_t i = 0; i < 2000000; ++i)
        {
        }
    }
}
