#include "ihandlers.h"
#include "SPImcpDAC.h"
#include "timer.h"
#include "build.h"

#include "wavetable_0.h"
#include "wavetable_1.h"
#include "wavetable_2.h"
#include "wavetable_3.h"
#include "wavetable_4.h"
#include "wavetable_5.h"
#include "wavetable_6.h"
#include "wavetable_7.h"

#ifdef WAVEFORM_INCLUDE_PURE_SINE
#include "wavetable_pure_sine1024.h"
#endif

// future expansion from 1024 to 2048 point wavetables
#define DDS_WAVETABLE_RESOLUTION 0x3ff
const int16_t *const wavetables[] = {
    #ifdef WAVEFORM_INCLUDE_PURE_SINE
        wavetable_sine1024, // pure sine
    #endif
        wavetable_0,
        wavetable_1,
        wavetable_2,
        wavetable_3,
        wavetable_4,
        wavetable_5,
        wavetable_6,
        wavetable_7,
};
const uint8_t num_wavetables = (sizeof(wavetables) / sizeof(wavetables[0]));

#if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE
volatile uint8_t interpolation_mode = 0;        // accessible from main loop
#endif

static const uint32_t MCP_DAC_BASE = 2048;

#define F_VOL_PIN (PIND & (1 << PORTD2))
#define F_PITCH_PIN (PINB & (1 << PORTB0))

volatile uint16_t vScaledVolume = 0;        // output level amplitude
volatile uint16_t vPointerIncrement = 0;    // phase accumulator increment

volatile uint16_t pitch = 0;                // Pitch value
volatile uint16_t pitch_counter = 0;        // Pitch counter
volatile uint16_t pitch_counter_l = 0;      // Last value of pitch counter

volatile bool volumeValueAvailable = 0;     // Volume read flag
volatile bool pitchValueAvailable = 0;      // Pitch read flag
volatile bool reenableInt1 = 0;             // reeanble Int1

volatile uint16_t vol;                      // Volume value
volatile uint16_t vol_counter = 0;          // volume counter
volatile uint16_t vol_counter_i = 0;        // Volume counter Timer 1
volatile uint16_t vol_counter_l;            // Last value of volume counter

volatile int16_t pitchCV;                   // Pitch CV value
volatile uint16_t volCV;                    // Volume CV value
volatile bool volumeCVAvailable;            // Volume CV flag
volatile bool pitchCVAvailable;             // Pitch CV flag

volatile uint16_t timer_overflow_counter;   // counter for frequency measurement
volatile uint8_t vWavetableSelector = 0;    // wavetable selector

static volatile uint16_t pointer = 0;       // Table pointer 22-bit: 16 bits integer + 6 bits fraction

#if DDS_DEBOUNCE_MODE == DDS_DEBOUNCE_MODE_LEGACY
static volatile uint8_t debounce_p, debounce_v = 0; // Counters for debouncing
#endif

/**
 * @brief Initializes hardware timers used for pitch frequency measurement and system timing.
 *
 * This function sets up:
 * - Timer1 in normal mode with input capture on rising edge at full 16 MHz clock rate,
 *     used to measure the frequency of the pitch oscillator via the ICP1 pin (PD5 / Digital 5).
 * - Timer0 with Arduino default Fast PWM mode and a 64× prescaler,
 *     which supports core timing functions like `millis()` and `delay()`.
 *
 * Timer1 Input Capture Interrupt is enabled to allow precise measurement of pitch events.
 * Timer0 Overflow Interrupt is also enabled, following Arduino's default timer configuration.
 *
 * @note
 * - Timer1 is configured to run without prescaling for maximum frequency resolution.
 * - The ICES1 bit ensures rising edge detection on ICP1 (PD5).
 * - No waveform output is enabled; timers are used strictly for counting.
 */
void ihInitialiseTimer() {
    // --- Timer1: Pitch frequency measurement (ICP1 on PD5) ---        
    /**
     * TCCR1A controls waveform generation mode and output compare behavior.
     * Setting it to 0 disables waveform output on OC1A/OC1B and enables normal counting mode.
     */
    TCCR1A = 0; ///< Normal mode, no PWM output
    
    /**
     * TCCR1B:
     * - ICES1: Input Capture Edge Select → 1 = capture on rising edge
     * - CS10:    Clock Select → 1 = no prescaler (Timer1 counts at full 16 MHz)
     */
    TCCR1B = (1 << ICES1) | (1 << CS10); ///< Rising edge capture, no prescaling

    /**
     * TIMSK1:
     * - ICIE1: Enable Timer1 Input Capture interrupt (TIMER1_CAPT_vect)
     */
    TIMSK1 = (1 << ICIE1); ///< Enable input capture interrupt

    // --- Timer0: Arduino core timebase (used by millis/delay) ---

    /**
     * TCCR0A:
     * - WGM01 + WGM00 = 1 → Fast PWM mode (Arduino default)
     */
    TCCR0A = 0x03; ///< Fast PWM mode

    /**
     * TCCR0B:
     * - CS01 + CS00 = 1 → Prescaler = clk/64 (Timer0 runs at 250 kHz)
     */
    TCCR0B = 0x03; ///< Clock /64 prescaler

    /**
     * TIMSK0:
     * - TOIE0: Enable Timer0 Overflow Interrupt (used by Arduino core)
     */
    TIMSK0 = 0x01; ///< Enable Timer0 overflow interrupt
}

/**
 * @brief Configures external interrupts for wave generator and volume measurement.
 *
 * This function sets up the External Interrupt Control Register (EICRA) to trigger interrupts
 * on rising edges for both INT0 (volume gate, F_VOL) and INT1 (sample clock / wave generator tick).
 * These are connected respectively to PD2 (INT0) and PD3 (INT1) on the ATmega328P.
 *
 * - INT0 is used to latch volume oscillator timing information from the flip-flop gate.
 * - INT1 triggers the main waveform ISR for audio output and pitch/volume tracking.
 *
 * External interrupts are enabled in the mask register (EIMSK), and a software flag
 * (`reenableInt1`) is set to allow re-arming INT1 from within the ISR after it's temporarily disabled.
 *
 * @note This setup is AVR-specific. On ARM MCUs like the DUE or STM32, similar behavior
 *             would be implemented with attachInterrupt() or direct NVIC/GPIO interrupt configuration.
 */
void ihInitialiseInterrupts() {
    // --- Configure interrupt trigger type for INT0 (PD2) and INT1 (PD3) ---

    /* EICRA – External Interrupt Control Register A
            ISC00 + ISC01 = 1 + 1 = Rising edge trigger for INT0 (Volume)
            ISC10 + ISC11 = 1 + 1 = Rising edge trigger for INT1 (Wave ISR)
    */
    EICRA = (1 << ISC00) | (1 << ISC01) | (1 << ISC11) | (1 << ISC10);

    reenableInt1 = true; // Internal flag to allow INT1 to be re-enabled in the main ISR

    /* EIMSK – External Interrupt Mask Register
            Enables INT0 (bit 0) and INT1 (bit 1)
    */
    EIMSK = (1 << INT0) | (1 << INT1);
}


/**
 * @brief Prepares Timer1 for direct pitch oscillator frequency measurement.
 *
 * This routine disables external interrupts and configures Timer1 to count freely,
 * using the full 16 MHz system clock, with overflow interrupt enabled.
 * It’s typically called during absolute pitch calibration or special diagnostic modes.
 *
 * The RF oscillator signal (VO_PITCH) enters on PD5 (ICP1 / T1), and its rising edge
 * transitions are captured by periodically reading the counter value (TCNT1).
 *
 * Configuration:
 * - Disables external INT0 and INT1 (EIMSK = 0)
 * - Timer1 normal mode with no waveform generation (TCCR1A = 0)
 * - Enables Timer1 Overflow Interrupt (used to measure longer periods)
 *
 * @note No prescaling is used for maximum resolution. The overflow ISR (TIMER1_OVF_vect)
 *             increments a software counter to extend timing range beyond 16 bits.
 */
void ihInitialisePitchMeasurement() {
    reenableInt1 = false;
    EIMSK = 0;                      ///< Disable External Interrupts INT0 and INT1
    TCCR1A = 0;                     ///< Timer1 Normal Mode, no PWM or compare output
    TIMSK1 = (1 << TOIE1);          ///< Enable Timer1 Overflow Interrupt (TIMER1_OVF_vect)
}

/**
 * @brief Configures Timer0 and Timer1 to perform volume oscillator measurement.
 *
 * This mode is used during calibration or diagnostics to capture the absolute frequency
 * of the volume RF oscillator (VO_VOL) using internal timers only.
 *
 * Configuration steps:
 * - Disables external interrupts (EIMSK = 0)
 * - Timer0 is used in Normal mode (TCCR0A = 0) and set to trigger compare-match interrupts.
 * - Timer1 is used as a free-running counter with prescaler 1024 to extend timing range.
 *
 * Volume signal transitions (VO_VOL) are detected by checking TCNT1 in relation
 * to Timer0 Compare Match A (OCR0A), which generates periodic samples.
 *
 * @note This is distinct from regular volume gating via F_VOL/INT0 and is more suited
 *             for precise, absolute measurements when direct oscillator observation is required.
 */
void ihInitialiseVolumeMeasurement() {
    reenableInt1 = false;

    EIMSK = 0;                      ///< Disable External Interrupts INT0 and INT1
    TIMSK1 = 0;                     ///< Disable Timer1 Input Capture and Overflow interrupts

    TCCR0A = 0;                     ///< Timer0 Normal mode (no PWM)
    TIMSK0 = (1 << OCIE0A);         ///< Enable Timer0 Compare Match A interrupt (TIMER0_COMPA_vect)
    OCR0A = 0xFF;                   ///< Set Compare Match A value to max (255)

    TCCR1A = 0;                     ///< Timer1 Normal mode
    TCCR1B = (1 << CS10) | (1 << CS12); ///< Prescaler = 1024 (clock = 15.625 kHz)
    TCCR1C = 0;                     ///< No forced compare
}


/**
 * @brief Main waveform interrupt handler triggered by SAMPLE_CLK (INT1).
 *
 * This ISR is invoked at the audio sample rate (31.25 kHz) and performs:
 * - DAC waveform update and pointer increment.
 * - Rising-edge detection of F_PITCH and F_VOL signals.
 * - Frequency capture using Timer1.
 * - CV output if enabled.
 *
 * ## Signal Input Explanation
 * - `F_PITCH_STATE` reads pin PB0 (digital pin 8) — the F_PITCH signal after flip-flop.
 *     Defined as: `(PINB & (1 << PORTB0))` → true when PB0 is HIGH.
 *
 * - `F_VOL_STATE` reads pin PD2 (digital pin 2) — the F_VOL signal after flip-flop.
 *     Defined as: `(PIND & (1 << PORTD2))` → true when PD2 is HIGH.
 *
 * These signals are fed from the logic divider+flip-flop gates for pitch and volume.
 * A rising edge indicates a new gated RF cycle suitable for timing measurement.
 *
 * - debounce method: 3-sample debounce followed by 2-sample confirmation.
 *
 * Externaly generated 31250 Hz Interrupt for WAVE generator (32us) 
 * 16MHz / 512 (2^9) = 31250 SAMPLE_CLK PD3
 * 
 * @note Keep interrupt duration below sample period (~32 µs).
 */
ISR(INT1_vect) {
    EIMSK &= ~ (1 << INT1);                     ///< Disable further external interrupts to prevent re-entry
    SPImcpDAClatch();                           ///< Latch previous DAC value before sending a new sample
    interrupts();                               ///< Re-enable nested interrupts to allow counter 1 interrupts

    uint32_t scaledSample;
    // extract the fractional offset from the 16.6 fixed-point phase accumulator (pointer).
    // Shifting by 6 matches the 64-step sub-sample precision — efficient and avoids floating-point.
    uint16_t offset = (pointer >> 6) & DDS_WAVETABLE_RESOLUTION; // 10-bit table index
    #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE
        switch(interpolation_mode) {
            case INTERPOLATION_LINEAR: {
    #endif
    #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE || INTERPOLATION_MODE == INTERPOLATION_LINEAR
        uint16_t frac = pointer & 0x3F; // 6-bit fractional part
        int16_t a = pgm_read_word_near(wavetables[vWavetableSelector] + offset);
        int16_t b = pgm_read_word_near(wavetables[vWavetableSelector] + ((offset + 1) & DDS_WAVETABLE_RESOLUTION));
        int16_t interp = a + (((int32_t)(b - a) * frac) >> 6);
        scaledSample = ((int32_t)interp * vScaledVolume) >> 16;
    #endif
    #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE
            } break;
        case INTERPOLATION_HERMITE: {
    #endif
    #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE || INTERPOLATION_MODE == INTERPOLATION_HERMITE
        int32_t frac = pointer & 0x3F;                  // 6-bit fractional part
        
        int16_t xm1 = pgm_read_word_near(wavetables[vWavetableSelector] + ((offset - 1) & DDS_WAVETABLE_RESOLUTION));
        int16_t x0  = pgm_read_word_near(wavetables[vWavetableSelector] + offset);
        int16_t x1  = pgm_read_word_near(wavetables[vWavetableSelector] + ((offset + 1) & DDS_WAVETABLE_RESOLUTION));
        int16_t x2  = pgm_read_word_near(wavetables[vWavetableSelector] + ((offset + 2) & DDS_WAVETABLE_RESOLUTION));

        int32_t t2 = (frac * frac) >> 6;
        int32_t t3 = (t2 * frac) >> 6;

        // Hermite coefficients (scaled to fixed-point)
        int16_t m0 = (x1 - xm1) >> 1;
        int16_t m1 = (x2 - x0) >> 1;

        int32_t h00 = (2 * t3 - 3 * t2 + 64);           // +64 simulates fixed-point +1.0 in Q6
        int32_t h10 = t3 - 2 * t2 + frac;
        int32_t h01 = -2 * t3 + 3 * t2;
        int32_t h11 = t3 - t2;

        int32_t interp = ((h00 * x0) + (h10 * m0) + (h01 * x1) + (h11 * m1)) >> 6;
        scaledSample = ((int32_t)interp * vScaledVolume) >> 16;
    #endif
    #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE
            } break;
        case INTERPOLATION_CATMULL_ROM: {
    #endif
    #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE || INTERPOLATION_MODE == INTERPOLATION_CATMULL_ROM
        int32_t frac = pointer & 0x3F; // 6-bit fractional part

        int16_t xm1 = pgm_read_word_near(wavetables[vWavetableSelector] + ((offset - 1) & DDS_WAVETABLE_RESOLUTION));
        int16_t x0  = pgm_read_word_near(wavetables[vWavetableSelector] + offset);
        int16_t x1  = pgm_read_word_near(wavetables[vWavetableSelector] + ((offset + 1) & DDS_WAVETABLE_RESOLUTION));
        int16_t x2  = pgm_read_word_near(wavetables[vWavetableSelector] + ((offset + 2) & DDS_WAVETABLE_RESOLUTION));

        // Catmull-Rom spline coefficients (scaled by 0.5, hence >> 1)
        int32_t a0 = ((-xm1 + 3*x0 - 3*x1 + x2) >> 1);
        int32_t a1 = ((2*xm1 - 5*x0 + 4*x1 - x2) >> 1);
        int32_t a2 = (x1 - xm1) >> 1;
        int32_t a3 = x0;

        // Evaluate cubic polynomial with fixed-point frac in [0, 63]
        int32_t interp = ((a0 * frac) >> 6);            // a0·t
        interp = ((interp + a1) * frac) >> 6;           // (a0·t + a1)·t
        interp = ((interp + a2) * frac) >> 6;           // ((...) + a2)·t
        interp = interp + a3;                           // + a3

        // Scale by volume (16-bit fractional)
        scaledSample = ((int32_t)interp * vScaledVolume) >> 16;
    #endif
    #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE
            } break;
        case INTERPOLATION_NONE: {
    #endif
    #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE || INTERPOLATION_MODE == INTERPOLATION_NONE
        int16_t waveSample = (int16_t)pgm_read_word_near(wavetables[vWavetableSelector] + offset);
        scaledSample = ((int32_t)waveSample * (uint32_t)vScaledVolume) >> 16;
    #endif
    #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE
            } break;
        default: break;
    }
    #endif
    SPImcpDACsend(scaledSample + MCP_DAC_BASE);
    pointer += vPointerIncrement;                           ///< Advance wavetable phase pointer
    incrementTimer();                                       ///< Update 32us system timer tick

    #if DDS_DEBOUNCE_MODE == DDS_DEBOUNCE_MODE_EDGE_DETECT
        // --- Rising-edge detection (modern) ---
        static bool lastPC = false;     ///< last pitch counter
        static bool lastVC = false;     ///< last volume counter

        bool currentPC = F_PITCH_PIN;   ///< Read F_PITCH signal on PB0 (digital pin 8)
        bool currentVC = F_VOL_PIN;     ///< Read F_VOL signal on PD2 (digital pin 2)

        // --- F_PITCH Rising Edge Detection (PB0)
        if (currentPC && !lastPC) {
            pitch_counter = ICR1;                            ///< Capture current Timer1 count
            pitch = pitch_counter - pitch_counter_l;         ///< Compute elapsed ticks
            pitch_counter_l = pitch_counter;                 ///< Store current count for next diff
            pitchValueAvailable = true;
        }
        lastPC = currentPC;

        // --- F_VOL Rising Edge Detection (PD2)
        if (currentVC && !lastVC) {
            vol_counter = vol_counter_i;                     ///< Volume measured indirectly via TCNT1
            vol = vol_counter - vol_counter_l;
            vol_counter_l = vol_counter;
            volumeValueAvailable = true;
        }
        lastVC = currentVC;
    #else
        // legacy debounce
        // PB0 == F_PITCH
        if (F_PITCH_PIN) { debounce_p++; }
        if (debounce_p == 3) {
            noInterrupts();
            pitch_counter = ICR1;                               ///< Get Timer-Counter 1 value
            pitch = pitch_counter - pitch_counter_l;            ///< Counter change since last interrupt -> pitch value
            pitch_counter_l = pitch_counter;                    ///< Set actual value as new last value
        } else if (debounce_p == 5) { pitchValueAvailable = true; }

        // PD2 == F_VOL
        if (F_VOL_PIN) { debounce_v++; }
        if (debounce_v == 3) {
            noInterrupts();
            vol_counter = vol_counter_i;                        ///< Get Timer-Counter 1 value
            vol = (vol_counter - vol_counter_l);                ///< Counter change since last interrupt
            vol_counter_l = vol_counter;                        ///< Set actual value as new last value
        } else if (debounce_v == 5) { volumeValueAvailable = true; }
    #endif
    
    #if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
        if (pitchCVAvailable) {
            /* 
            * At the very end to limit potential interference with sound generation
            * Priority on pitchCV, volumeCV if occurring at the same moment might be delayed by 32us without harm,
            * but it's better to do max. 1 additional SPI transaction per interrupt do leave enough headroom for the loop.
            */
            SPImcpDAC3Asend(pitchCV);                       // Send result to DAC (pitchCV out) (5.5 us)
            pitchCVAvailable = false;                       // Reset flag
        } else if (volumeCVAvailable) {
            SPImcpDAC3Bsend(volCV);                         // Send result to DAC (volCV out) (5.5 us)
            volumeCVAvailable = false;                      // Reset flag
        }
    #endif

    noInterrupts();
    if (reenableInt1) { EIMSK |= (1 << INT1); } ///< Re-enable external interrupts (INT1) for next SAMPLE_CLK pulse
}

/* VOLUME read - interrupt service routine for capturing volume counter value */
ISR(INT0_vect) {
    vol_counter_i = TCNT1;
    #if DDS_DEBOUNCE_MODE == DDS_DEBOUNCE_MODE_LEGACY
    debounce_v = 0;
    #endif
}

/* PITCH read - interrupt service routine for capturing pitch counter value */
ISR(TIMER1_CAPT_vect) {
    #if DDS_DEBOUNCE_MODE == DDS_DEBOUNCE_MODE_LEGACY
    debounce_p = 0;
    #endif
}

/* PITCH read absolute frequency for calibration measurement */
ISR(TIMER0_COMPA_vect) {
    timer_overflow_counter++;
}

/* VOLUME read absolute frequency for calibration measurement */
ISR(TIMER1_OVF_vect) {
    timer_overflow_counter++;
}
