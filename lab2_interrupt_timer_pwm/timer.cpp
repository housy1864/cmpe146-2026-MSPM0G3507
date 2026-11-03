#include "../hal/timer.hpp"

#include <limits>
#include <stdexcept>

namespace{

    constexpr std::uint32_t BIT(std::uint32_t p_position) {
        return 1u << p_position;
    }

    constexpr std::uintptr_t TIMG6_BASE  = 0x4086'8000UL;
    constexpr std::uintptr_t TIMG8_BASE  = 0x4009'0000UL;
    constexpr std::uintptr_t TIMG12_BASE = 0x4087'0000UL;

    constexpr std::uintptr_t IOMUX_BASE = 0x4042'8000UL;

    // GPTIMER register offsets
    constexpr std::uintptr_t PWREN  = 0x800;
    constexpr std::uintptr_t CLKDIV = 0x1000;
    constexpr std::uintptr_t CLKSEL = 0x1008;
    constexpr std::uintptr_t CPS    = 0x110C;

    constexpr std::uintptr_t ODIS   = 0x1104;
    constexpr std::uintptr_t CCPD   = 0x1100;

    constexpr std::uintptr_t CTR    = 0x1800;
    constexpr std::uintptr_t CTRCTL = 0x1804;
    constexpr std::uintptr_t LOAD   = 0x1808;

    constexpr std::uintptr_t CC0    = 0x1810;
    constexpr std::uintptr_t CCCTL0 = 0x1830;

    constexpr std::uintptr_t OCTL0  = 0x1850;
    constexpr std::uintptr_t CCACT0 = 0x1870;

    // MSPM0 BUSCLK used by this lab.
    constexpr std::uint32_t TIMER_CLOCK_HZ = 32'000'000;
    constexpr std::uint32_t CHANNEL_COUNT = 4;
    constexpr std::uint32_t DUTY_CYCLE_MAX = 65'535;

    // PWREN:
    // key = 0x26 in upper byte
    // enable = bit 0
    constexpr std::uint32_t PWREN_ENABLE = 0x2600'0001UL;


    // CLKSEL BUSCLK = bit 3
    constexpr std::uint32_t CLKSEL_BUSCLK = 0x0000'0008UL;


    // CCLKCTL enable
    constexpr std::uintptr_t CCLKCTL = 0x1108;
    constexpr std::uint32_t CCLK_ENABLE = 0x0000'0001UL;


    // Counter control
    constexpr std::uint32_t CTRCTL_ENABLE = BIT(0);
    // REPEAT is a field at bits 3:1; field encoding 1 repeats after zero.
    constexpr std::uint32_t CTRCTL_REPEAT = 1u << 1;
    // CM is bits 5:4: 0=down, 1=up/down, 2=up.
    constexpr std::uint32_t CTRCTL_UP = 2u << 4;
    constexpr std::uint32_t CTRCTL_DOWN = 0u << 4;
    constexpr std::uint32_t PINCM_OUTPUT_ENABLE = BIT(7);
    constexpr std::uint32_t PIN_FUNCTION_3 = 3u;
    constexpr std::uint32_t PIN_FUNCTION_5 = 5u;

    // IOMUX configuration
    // PINCM numbering starts at 1, so:
    // PB22  -> PINCM50 -> PF 3 -> TIMG8_C1
    // PB26  -> PINCM57 -> PF 5 -> TIMG6_C0
    // PB27  -> PINCM58 -> PF 5 -> TIMG6_C1
    struct pwm_pin_route {
       std::uintptr_t timer_base;
       std::uint8_t channel;
       std::uintptr_t pin_cm;
       std::uint32_t function;
    };

    // Supported board LED routes from the MSPM0G3507 pin-function table.
    constexpr pwm_pin_route PWM_PIN_ROUTES[] = {
        {TIMG8_BASE, 1, IOMUX_BASE + 50u * 4u,
         PINCM_OUTPUT_ENABLE | PIN_FUNCTION_3}, // PB22, blue
        {TIMG6_BASE, 0, IOMUX_BASE + 57u * 4u,
         PINCM_OUTPUT_ENABLE | PIN_FUNCTION_5}, // PB26, red
        {TIMG6_BASE, 1, IOMUX_BASE + 58u * 4u,
         PINCM_OUTPUT_ENABLE | PIN_FUNCTION_5}, // PB27, green
    };

    // PWM output control.
    // OCTL CCPO = 0 means use the timer's signal generator.
    constexpr std::uint32_t OCTL_FUNCTION_VALUE = 0;


    // CCACT:
    // LOAD event -> HIGH
    // compare-down event -> LOW
    //
    // This creates:
    //       LOAD ─> HIGH
    //       compare ─> LOW
    //       zero ─> next LOAD
    //
    // Therefore the percentage of time HIGH is the duty cycle.

    constexpr std::uint32_t CCACT_LOAD_HIGH = 1u << 3;
    constexpr std::uint32_t CCACT_COMPARE_DOWN_LOW = 2u << 6;

    constexpr std::uint32_t COUNTER_MAX =
        std::numeric_limits<std::uint32_t>::max();

    std::uint32_t calculate_period(std::uint32_t p_frequency)
    {
        if (p_frequency == 0 || p_frequency > TIMER_CLOCK_HZ) {
            throw std::invalid_argument("PWM frequency must be within timer clock range");
        }
        auto const period = TIMER_CLOCK_HZ / p_frequency;
        if (period < 2) {
            throw std::invalid_argument("PWM period must be at least two timer ticks");
        }
        return period;
    }

    pwm_pin_route find_pin_route(std::uintptr_t p_timer_base,
                                 std::uint8_t p_channel)
    {
        for (auto const& route : PWM_PIN_ROUTES) {
            if (route.timer_base == p_timer_base && route.channel == p_channel) {
                return route;
            }
        }
        throw std::invalid_argument("No board pin route for this timer channel");
    }

    // Convert integer register address into a volatile 32-bit register.
    inline volatile std::uint32_t* reg(
    std::uintptr_t p_address)
    {
    return reinterpret_cast<volatile std::uint32_t*>(p_address);
    }

    void configure_timer_clock(std::uintptr_t p_base)
    {
    // Enable timer peripheral power.
    *reg(p_base + PWREN) = PWREN_ENABLE;

    // Select BUSCLK.
    *reg(p_base + CLKSEL) = CLKSEL_BUSCLK;

    // BUSCLK / 1.
    *reg(p_base + CLKDIV) = 0;

    // Timer prescaler / 1.
    *reg(p_base + CPS) = 0;

    // Enable timer counter clock.
    *reg(p_base + CCLKCTL) = CCLK_ENABLE;
    }

    void configure_pwm_output(
    std::uintptr_t p_base,
    std::uint8_t p_channel)
    {
    if (p_channel >= CHANNEL_COUNT) {
        throw std::invalid_argument("GPTIMER channel must be between 0 and 3");
    }

    auto const cc_offset =
        static_cast<std::uintptr_t>(p_channel) * sizeof(std::uint32_t);

    auto const ccctl_offset =
        static_cast<std::uintptr_t>(p_channel) * sizeof(std::uint32_t);

    auto const octl_offset =
        static_cast<std::uintptr_t>(p_channel) * sizeof(std::uint32_t);

    auto const ccact_offset =
        static_cast<std::uintptr_t>(p_channel) * sizeof(std::uint32_t);

    // Start with output disabled.
    // ODIS=1 forces this output low while it is configured.
    auto const channel_mask = BIT(p_channel);
    *reg(p_base + ODIS) |= channel_mask;

    // PWM output is controlled by the signal generator.
    *reg(p_base + OCTL0 + octl_offset) =
        OCTL_FUNCTION_VALUE;

    // Load -> HIGH
    // Compare down -> LOW
    *reg(p_base + CCACT0 + ccact_offset) =
        CCACT_LOAD_HIGH |
        CCACT_COMPARE_DOWN_LOW;

    // Compare channel control.
    // Configure this CCP channel as an output.
    *reg(p_base + CCPD) |= channel_mask;
    // 0 means normal compare operation.
    *reg(p_base + CCCTL0 + ccctl_offset) = 0;

    // Clear compare initially.
    *reg(p_base + CC0 + cc_offset) = 0;

    // Enable the physical CCP output.
    *reg(p_base + ODIS) &= ~channel_mask;
    
    }

}//namespace

namespace lab2
{

    mspm0g_steady_clock::mspm0g_steady_clock()
    {
    configure_timer_clock(TIMG12_BASE);

    // 1 second = 32,000,000 timer ticks.
    //
    // TIMG12 is 32-bit, so it can run for:
    //
    // 2^32 / 32,000,000 ≈ 134 seconds
    //
    // before wrapping.
    *reg(TIMG12_BASE + LOAD) = COUNTER_MAX;

    // Periodic up-counting.
    //
    // ENABLE = 1
    // REPEAT = 1
    // UP     = 1
    *reg(TIMG12_BASE + CTRCTL) =
        CTRCTL_ENABLE |
        CTRCTL_REPEAT |
        CTRCTL_UP;
    }


    std::uint32_t
    mspm0g_steady_clock::driver_frequency()
    {
    return TIMER_CLOCK_HZ;
    }


    std::uint64_t
    mspm0g_steady_clock::driver_uptime()
    {
    std::uint32_t const current =
        *reg(TIMG12_BASE + CTR);

    // Detect 32-bit wrap-around.
    if (current < m_last_counter) {
        ++m_overflow_count;
    }

    m_last_counter = current;

    return
        (m_overflow_count << 32) |
        static_cast<std::uint64_t>(current);
    }


    mspm0g_pwm::mspm0g_pwm(
    std::uintptr_t p_timer_base,
    std::uint8_t p_channel,
    std::uint32_t p_frequency)
    : m_timer_base(p_timer_base)
    , m_channel(p_channel)
    , m_frequency(p_frequency)
    , m_period(TIMER_CLOCK_HZ / p_frequency)
    {
    configure_timer_clock(m_timer_base);

    // Configure RGB pin mux.
    //
    // TIMG6:
    //   C0 -> PB26
    //   C1 -> PB27
    //
    // TIMG8:
    //   C1 -> PB22
    auto const pin_route = find_pin_route(m_timer_base, m_channel);
    *reg(pin_route.pin_cm) = pin_route.function;

    configure_pwm_output(
        m_timer_base,
        m_channel);

    // Period.
    //
    // Timer counts from LOAD down to zero.
    *reg(m_timer_base + LOAD) =
        m_period - 1;

    // Initialize the first cycle from LOAD so it has the configured period.
    *reg(m_timer_base + CTR) = m_period - 1;

    // Start from the LOAD value.
    *reg(m_timer_base + CTRCTL) =
        CTRCTL_REPEAT |
        CTRCTL_DOWN;

    // Enable after configuration.
    *reg(m_timer_base + CTRCTL) =
        CTRCTL_ENABLE |
        CTRCTL_REPEAT |
        CTRCTL_DOWN;
    }


    std::uint32_t
    mspm0g_pwm::driver_frequency()
    {
    return m_frequency;
    }


    void
    mspm0g_pwm::driver_duty_cycle(
    std::uint16_t p_duty_cycle)
    {
    //
    // p_duty_cycle:
    //
    // 0      = 0%
    // 32768  ≈ 50%
    // 65535  = 100%
    //
    // For down-count PWM:
    //
    // compare = period * (1 - duty)
    //
    // Example:
    //
    // period = 32000
    // duty   = 25%
    //
    // compare = 32000 * 0.75
    //         = 24000
    //

    std::uint32_t const period =
        m_period;

    std::uint32_t const compare =
        static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(period - 1) *
        (DUTY_CYCLE_MAX) - p_duty_cycle)) /
        DUTY_CYCLE_MAX);

    auto const cc_offset =
        static_cast<std::uintptr_t>(m_channel) * sizeof(std::uint32_t);

    *reg(m_timer_base + CC0 + cc_offset) =
        compare;
    }

}  // namespace lab2