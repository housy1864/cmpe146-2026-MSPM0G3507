#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <chrono>

#include "../hal/timer.hpp"
#include "../hal/timer_util.hpp"

// Test double for lab2::steady_clock. Does not touch real hardware - it just
// increments a counter every time uptime() is called, as if a timer tick had
// elapsed. Useful for exercising delay logic before your real Timer_A-backed
// steady_clock exists.
/*class fake_steady_clock : public lab2::steady_clock //this turns the hardware timer to a useable source
{
public:
  fake_steady_clock() = default;

private:
  std::uint32_t driver_frequency() override
  {
    // Use a small frequency to reduce the amount of ticks needed
    // for the delay function.
    return 1'000'000;
  }

  std::uint64_t driver_uptime() override
  {
    return m_count++;
  }

  std::uint64_t m_count = 0;
};*/

namespace {
constexpr std::uint16_t DUTY_CYCLE_MAX = 65'535;
constexpr std::uint16_t DUTY_CYCLE_MIN = 0;
constexpr int COLOR_STEPS = 10;
constexpr double HALF_TURN_RADIANS = 3.14159265358979323846;
constexpr double NORMALIZED_WAVE_MAX = 1.0;
constexpr double NORMALIZED_WAVE_RANGE = 2.0;
constexpr std::uint32_t PWM_FREQUENCY_HZ = 1'000;
constexpr auto COLOR_FRAME_DELAY = std::chrono::milliseconds(100);
}

int main()
{
  using namespace std::chrono_literals;
  std::printf("Hello, World\n");

  // TODO(lab2, step 1): Implement lab2::steady_clock

  lab2::mspm0g_steady_clock clock;

  std::printf(
    "Clock frequency = %" PRIu32 " Hz\n",
    clock.frequency());

  // TODO(lab2, step 2): Pass your steady clock to lab2::delay() from
  // hal/timer_util.hpp and test it with printf or blinking an LED - your
  // choice. Put a printf on either side of the delay and confirm the gap
  // between them matches the duration you asked for. If a 1s delay is not
  // taking 1 second, your frequency() is wrong.

  std::printf("Before 1 second delay\n");

  lab2::delay(clock, 1s);

  std::printf("After 1 second delay\n");

  // TODO(lab2, step3): Implement lab2::pwm using what you learned from
  // lab2::steady_clock

  // PB26 = Red   = TIMG6_C0
  // PB27 = Green = TIMG6_C1
  // PB22 = Blue  = TIMG8_C1

  lab2::mspm0g_pwm red(
    lab2::timg6_base_address,
    0,
    PWM_FREQUENCY_HZ);

  lab2::mspm0g_pwm green(
    lab2::timg6_base_address,
    1,
    PWM_FREQUENCY_HZ);

  lab2::mspm0g_pwm blue(
    lab2::timg8_base_address,
    1,
    PWM_FREQUENCY_HZ);

  // TODO(lab2, step4): Test against an LED and see if you can control the
  // brightness

  //red.duty_cycle(65535);
  //green.duty_cycle(0);
  //blue.duty_cycle(0);

  //fake_steady_clock clock;

  /*while (true) {

    lab2::delay(clock, 1s);
    std::printf("Sleep 1\n");
    lab2::delay(clock, 1s);
    std::printf("Sleep 2\n");
    lab2::delay(clock, 1s);
    std::printf("Sleep 3\n");
    // TODO(lab2, step 5): Use the steady clock together with your PWM driver to
    // animate the RGB LED as a continuous color wheel, as described in
    // README.md.
  }*/

  while (true) {

    for (int i = 0; i <= COLOR_STEPS; i++)
    {
        double t = static_cast<double>(i) / static_cast<double>(COLOR_STEPS);

        double smooth =
            (NORMALIZED_WAVE_MAX - std::cos(HALF_TURN_RADIANS * t)) /
            NORMALIZED_WAVE_RANGE;

        red.duty_cycle(DUTY_CYCLE_MAX);

        green.duty_cycle(
            static_cast<std::uint16_t>(static_cast<double>(DUTY_CYCLE_MAX) * smooth)
        );

        blue.duty_cycle(DUTY_CYCLE_MIN);

        lab2::delay(clock, COLOR_FRAME_DELAY);
    }

    for (int i2 = 1; i2 <= COLOR_STEPS; i2++)
    {
        double t = static_cast<double>(i2) / static_cast<double>(COLOR_STEPS);

        double smooth =
            (NORMALIZED_WAVE_MAX - std::cos(HALF_TURN_RADIANS * t)) /
            NORMALIZED_WAVE_RANGE;

        red.duty_cycle(
          static_cast<std::uint16_t>(static_cast<double>(DUTY_CYCLE_MAX) * (NORMALIZED_WAVE_MAX-smooth))
        );

        green.duty_cycle(DUTY_CYCLE_MAX);

        blue.duty_cycle(DUTY_CYCLE_MIN);

        lab2::delay(clock, COLOR_FRAME_DELAY);
    }

    for (int j = 0; j <= COLOR_STEPS; j++)
    {
        double t = static_cast<double>(j) / static_cast<double>(COLOR_STEPS);

        double smooth =
            (NORMALIZED_WAVE_MAX - std::cos(HALF_TURN_RADIANS * t)) /
            NORMALIZED_WAVE_RANGE;

        red.duty_cycle(DUTY_CYCLE_MIN);

        green.duty_cycle(DUTY_CYCLE_MAX);

        blue.duty_cycle(
          static_cast<std::uint16_t>(static_cast<double>(DUTY_CYCLE_MAX) * smooth)
        );

        lab2::delay(clock, COLOR_FRAME_DELAY);
    }

    for (int j2 = 0; j2 <= COLOR_STEPS; j2++)
    {
        double t = static_cast<double>(j2) / static_cast<double>(COLOR_STEPS);

        double smooth =
            (NORMALIZED_WAVE_MAX - std::cos(HALF_TURN_RADIANS * t)) / NORMALIZED_WAVE_RANGE;

        red.duty_cycle(DUTY_CYCLE_MIN);

        green.duty_cycle(
          static_cast<std::uint16_t>(static_cast<double>(DUTY_CYCLE_MAX) * (NORMALIZED_WAVE_MAX-smooth))
        );

        blue.duty_cycle(DUTY_CYCLE_MAX);

        lab2::delay(clock, COLOR_FRAME_DELAY);
    }

    for (int z = 0; z <= COLOR_STEPS; z++)
    {
        double t = static_cast<double>(z) / static_cast<double>(COLOR_STEPS);

        double smooth =
            (NORMALIZED_WAVE_MAX - std::cos(HALF_TURN_RADIANS * t)) /
            NORMALIZED_WAVE_RANGE;

        red.duty_cycle(
          static_cast<std::uint16_t>(static_cast<double>(DUTY_CYCLE_MAX) * smooth)
        );

        green.duty_cycle(DUTY_CYCLE_MIN);

        blue.duty_cycle(DUTY_CYCLE_MAX);

        lab2::delay(clock, COLOR_FRAME_DELAY);
    }

    for (int z2 = 0; z2 <= COLOR_STEPS; z2++)
    {
        double t = static_cast<double>(z2) / static_cast<double>(COLOR_STEPS);

        double smooth =
            (NORMALIZED_WAVE_MAX - std::cos(HALF_TURN_RADIANS * t)) /
            NORMALIZED_WAVE_RANGE;

        red.duty_cycle(DUTY_CYCLE_MAX);

        green.duty_cycle(DUTY_CYCLE_MIN);

        blue.duty_cycle(
          static_cast<std::uint16_t>(static_cast<double>(DUTY_CYCLE_MAX) * (NORMALIZED_WAVE_MAX-smooth))
        );

        lab2::delay(clock, COLOR_FRAME_DELAY);
    }
    
  }
}