#include "pwm.h"
#include "shared.h"     /* RCC struct */
#include "Bit_Math.h"

/* ─────────────────────────────────────────
 * PWM_Init()
 * ─────────────────────────────────────────
 * Configures TIM1 Channel 1 on PA8 for
 * 10 kHz PWM motor simulation output.
 *
 * Register-level steps:
 *   1. Clock gating  → RCC AHB1ENR (GPIOA)
 *                    → RCC APB2ENR (TIM1)
 *   2. GPIO PA8      → MODER=AF, OSPEEDR=High, AFRH=AF1
 *   3. TIM1 config   → PSC, ARR, CCMR1, CCER, BDTR, CR1
 *   4. Start counter → CR1.CEN = 1
 *
 * Why BDTR.MOE?
 *   TIM1 is an "advanced-control" timer.
 *   Its outputs are disabled by default as a
 *   safety feature (break input logic).
 *   MOE (Main Output Enable) must be set to
 *   allow any PWM signal to appear on the pin.
 * ───────────────────────────────────────── */
void PWM_Init(void)
{
    /* ── Step 1: Enable peripheral clocks ── */

    /* GPIOA clock on AHB1 (bit 0) */
    SET_BIT(RCC->AHB1ENR, 0);

    /* TIM1 clock on APB2 (bit 0) */
    SET_BIT(RCC->APB2ENR, 0);

    /* ── Step 2: Configure PA8 → AF1 (TIM1_CH1) ── */

    /* PA8 is in the high nibble of GPIOA registers.
     * MODER bits [17:16] for pin 8 → set to 10 (AF mode) */
    volatile u32 *GPIOA_MODER   = (volatile u32 *)(0x40020000UL + 0x00);
    volatile u32 *GPIOA_OSPEEDR = (volatile u32 *)(0x40020000UL + 0x08);
    volatile u32 *GPIOA_AFRH   = (volatile u32 *)(0x40020000UL + 0x24);

    /* Clear MODER[17:16], then set to 10 (Alternate Function) */
    *GPIOA_MODER &= ~(0x3UL << 16);
    *GPIOA_MODER |=  (0x2UL << 16);

    /* Set OSPEEDR[17:16] = 10 → High speed (enough for 10kHz) */
    *GPIOA_OSPEEDR &= ~(0x3UL << 16);
    *GPIOA_OSPEEDR |=  (0x2UL << 16);

    /* AFRH controls pins 8–15.
     * Pin 8 → AFRH bits [3:0] → set to 0001 (AF1 = TIM1_CH1) */
    *GPIOA_AFRH &= ~(0xFUL << 0);
    *GPIOA_AFRH |=  (0x1UL << 0);   /* AF1 */

    /* ── Step 3: Configure TIM1 ── */

    /* Disable counter while configuring */
    CLEAR_BIT(PWM_TIM->CR1, TIM_CR1_CEN);

    /* Set prescaler: divides 16MHz by (15+1) = 1 MHz tick */
    PWM_TIM->PSC = PWM_PSC;

    /* Set auto-reload: 100 ticks per PWM period → 1MHz/100 = 10kHz */
    PWM_TIM->ARR = PWM_ARR;

    /* Repetition counter = 0 (not needed for basic PWM) */
    PWM_TIM->RCR = 0u;

    /* ── CCMR1: Configure CH1 as PWM output ──
     *
     * CC1S  [1:0] = 00  → CH1 is output (not capture)
     * OC1PE [3]   = 1   → Preload enable (CCR1 buffered)
     * OC1M  [6:4] = 110 → PWM Mode 1
     *   (output high while CNT < CCR1, low otherwise)
     */
    PWM_TIM->CCMR1 = 0u;   /* Clear first */
    PWM_TIM->CCMR1 |= TIM_OC_PWM_MODE1;          /* OC1M = 110        */
    SET_BIT(PWM_TIM->CCMR1, TIM_CCMR1_OC1PE);    /* OC1PE = 1         */

    /* ── CCER: Enable CH1 output, active-high ──
     *
     * CC1E [0] = 1  → Enable CH1 output
     * CC1P [1] = 0  → Active high (pin HIGH when CNT < CCR1)
     */
    PWM_TIM->CCER = 0u;
    SET_BIT(PWM_TIM->CCER, TIM_CCER_CC1E);

    /* ── Start with motor STOPPED (0% duty cycle) ── */
    PWM_TIM->CCR1 = PWM_DUTY_STOP;

    /* ── CR1: Enable auto-reload preload ──
     * ARPE [7] = 1 → ARR register is buffered
     * (prevents glitches when ARR is updated)     */
    SET_BIT(PWM_TIM->CR1, TIM_CR1_ARPE);

    /* ── BDTR: Enable Main Output (REQUIRED for TIM1) ──
     * MOE [15] = 1 → All PWM outputs enabled
     * Without this, TIM1 CH1 stays LOW regardless
     * of CCMR1 or CCER settings.                  */
    SET_BIT(PWM_TIM->BDTR, TIM_BDTR_MOE);

    /* ── Generate an update event to load PSC and ARR ──
     * Writing UG bit forces an update:
     *   - PSC and ARR shadow registers are refreshed
     *   - Counter resets to 0                       */
    SET_BIT(PWM_TIM->EGR, 0);   /* UG bit = bit 0 */

    /* ── Step 4: Start the counter ── */
    SET_BIT(PWM_TIM->CR1, TIM_CR1_CEN);
}

/* ─────────────────────────────────────────
 * PWM_SetDuty()
 * ─────────────────────────────────────────
 * Updates the CCR1 (capture/compare register)
 * to change the motor's PWM duty cycle.
 *
 * Since OC1PE=1 (preload enabled), the new
 * CCR1 value takes effect at the NEXT update
 * event (next PWM period), preventing glitches.
 *
 *  duty=0  → CCR1=0  → 0/100  = 0%   STOP
 *  duty=20 → CCR1=20 → 20/100 = 20%  SLOW
 *  duty=99 → CCR1=99 → 99/100 = 99%  FULL
 * ───────────────────────────────────────── */
void PWM_SetDuty(u8 duty)
{
    /* Clamp to valid range [0, ARR] */
    if (duty > PWM_ARR)
    {
        duty = (u8)PWM_ARR;
    }

    PWM_TIM->CCR1 = (u32)duty;
}