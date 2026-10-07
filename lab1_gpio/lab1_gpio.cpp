#include <cstdio>
#include "../third_party/span/span.hpp"
#include <array>

#include "../hal/gpio.hpp"

std::array<int, 64> my_buffer;

[[gnu::noinline]]
void foo(std::span<int> p_data)
{
  int volatile counter = 15;
  for (int volatile i = 0; i < my_buffer.size(); i++) {
    my_buffer[i] = counter++;
    std::printf("Count = %d\n", counter);
  }
}


class mspm0g_outpin : public lab1::output_pin
{
public:
  mspm0g_outpin(std::uint8_t p_port, std::uint8_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
    // Enable GPIO peripheral
    *get_reg(0x800) = 0x26000001;

    // Enable GPIO output
    *get_reg(0x12D0) = pin_mask();
  }

  std::uint8_t m_port;
  std::uint8_t m_pin;

private:
  std::uint32_t volatile* get_reg(std::ptrdiff_t p_offset)
  {
    if (m_port == 0) {
      return reinterpret_cast<std::uint32_t volatile*>(
          0x400A'0000UL + p_offset);
    }

    return reinterpret_cast<std::uint32_t volatile*>(
        0x400A'2000UL + p_offset);
  }

  std::uint32_t pin_mask() const
  {
    return (1u << m_pin);
  }

  std::uint32_t volatile* get_iomux_reg()
  {
    constexpr std::uintptr_t iomux_base = 0x40428000UL;

    std::uint32_t pin_cm;

    if (m_port == 0) {
      switch (m_pin) {
        case 18: pin_cm = 40; break;
        default: pin_cm = 0; break;
      }
    }
    else {
      switch (m_pin) {
        case 22: pin_cm = 50; break;
        case 26: pin_cm = 57; break;
        case 27: pin_cm = 58; break;
        default: pin_cm = 0; break;
      }
    }

    return reinterpret_cast<std::uint32_t volatile*>(
        iomux_base + (pin_cm * 4));
  }

  bool driver_configure(settings const& p_settings) override
  {
    // This is the value that we experimentally established
    // is making the RGB LED output work.
    std::uint32_t value = 0x40081;

    if (p_settings.resistor == lab1::pin_resistor::pull_up) {
      value |= 0x20000;
    }
    else if (p_settings.resistor == lab1::pin_resistor::pull_down) {
      value |= 0x10000;
    }

    if (p_settings.open_drain) {
      value |= 0x02000000;
    }

    *get_iomux_reg() = value;

    return true;
  }

  void driver_level(bool p_high) override
  {
    if (p_high) {
      *get_reg(0x1290) = pin_mask();
    }
    else {
      *get_reg(0x12A0) = pin_mask();
    }
  }

  bool driver_level() override
  {
    return (*get_reg(0x1280) & pin_mask()) != 0;
  }
};


class mspm0g_inpin : public lab1::input_pin
{
public:
  mspm0g_inpin(std::uint8_t p_port, std::uint8_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
    // Enable GPIO peripheral
    *get_reg(0x800) = 0x26000001;

    // Initial IOMUX configuration.
    // driver_configure() will configure it again.
    *get_iomux_reg() = 0x40401;
  }

private:
  std::uint8_t m_port;
  std::uint8_t m_pin;

  std::uint32_t pin_mask() const
  {
    return (1u << m_pin);
  }

  std::uint32_t volatile* get_reg(std::ptrdiff_t p_offset)
  {
    if (m_port == 0) {
      return reinterpret_cast<std::uint32_t volatile*>(
          0x400A'0000UL + p_offset);
    }

    return reinterpret_cast<std::uint32_t volatile*>(
        0x400A'2000UL + p_offset);
  }

  std::uint32_t volatile* get_iomux_reg()
  {
    constexpr std::uintptr_t iomux_base = 0x40428000UL;

    std::uint32_t pin_cm;

    if (m_port == 0) {
      switch (m_pin) {
        case 18: pin_cm = 40; break;
        default: pin_cm = 0; break;
      }
    }
    else {
      switch (m_pin) {
        case 21: pin_cm = 49; break;
        default: pin_cm = 0; break;
      }
    }

    return reinterpret_cast<std::uint32_t volatile*>(
        iomux_base + (pin_cm * 4));
  }

  bool driver_configure(settings const& p_settings) override
  {
    // We established experimentally that input enable
    // must be included here for S1 to work.
    std::uint32_t value = 0x40081;

    if (p_settings.resistor == lab1::pin_resistor::pull_up) {
      value |= 0x20000;
    }
    else if (p_settings.resistor == lab1::pin_resistor::pull_down) {
      value |= 0x10000;
    }

    *get_iomux_reg() = value;

    return true;
  }

  bool driver_level() override
  {
    return (*get_reg(0x1380) & pin_mask()) != 0;
  }
};


class fake_safety_system
{
public:
  fake_safety_system(lab1::output_pin* p_control)
    : m_control(p_control)
  {
    m_control->configure(
        {.resistor = lab1::pin_resistor::none,
         .open_drain = false});
  }

  void invoke_control() volatile
  {
    m_control->level(false);
  }

private:
  lab1::output_pin* m_control = nullptr;
};


lab1::output_pin& get_ctrl_pin()
{
  static mspm0g_outpin ctrl(0, 0);
  return ctrl;
}


int main()
{
  // =========================
  // RGB LED2
  // =========================

  mspm0g_outpin red(1, 26);
  mspm0g_outpin green(1, 27);
  mspm0g_outpin blue(1, 22);

  red.configure(
      {.resistor = lab1::pin_resistor::none,
       .open_drain = false});

  green.configure(
      {.resistor = lab1::pin_resistor::none,
       .open_drain = false});

  blue.configure(
      {.resistor = lab1::pin_resistor::none,
       .open_drain = false});


  // =========================
  // Push buttons
  // =========================

  mspm0g_inpin s1(0, 18);
  mspm0g_inpin s2(1, 21);

  // S1 = PA18, external pull-down
  s1.configure(
      {.resistor = lab1::pin_resistor::pull_down});

  // S2 = PB21, internal pull-up
  s2.configure(
      {.resistor = lab1::pin_resistor::pull_up});


  // =========================
  // Main loop
  // =========================

  while (true) {

    bool pressed_s1 = s1.level();
    bool pressed_s2 = !s2.level();

    if (pressed_s1 && pressed_s2) {

      // Both pressed → BLUE
      red.level(false);
      green.level(false);
      blue.level(true);

    }
    else if (pressed_s1) {

      // S1 → RED
      red.level(true);
      green.level(false);
      blue.level(false);

    }
    else if (pressed_s2) {

      // S2 → GREEN
      red.level(false);
      green.level(true);
      blue.level(false);

    }
    else {

      // Neither → OFF
      red.level(false);
      green.level(false);
      blue.level(false);
    }
  }
}