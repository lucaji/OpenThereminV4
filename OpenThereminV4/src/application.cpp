#include "application.h"
#include "SPImcpDAC.h"
#include "ihandlers.h"
#include "timer.h"
#include "EEPROM.h"

#define BUTTON_PIN 6
#define HW_BUTTON_STATE         (PIND & (1<<PORTD6))
#define HW_BUTTON_PRESSED       (HW_BUTTON_STATE == LOW)
#define HW_BUTTON_RELEASED      (HW_BUTTON_STATE != LOW)

#define LED_BLUE_PIN 18
#define HW_LED_BLUE_ON          (PORTC |= (1<<PORTC4))
#define HW_LED_BLUE_OFF         (PORTC &= ~(1<<PORTC4))
#define HW_LED_BLUE_TOGGLE      (PORTC = PORTC ^ (1<<PORTC4))

#define LED_RED_PIN 19
#define HW_LED_RED_ON           (PORTC |= (1<<PORTC5))
#define HW_LED_RED_OFF          (PORTC &= ~(1<<PORTC5))
#define HW_LED_RED_TOGGLE       (PORTC = PORTC ^ (1<<PORTC5))

#define GATE_PULLUP             (DDRC &= ~(1<<PORTC2)); (PORTC |= (1<<PORTC2))
#define GATE_SENSE              (PINC & (1<<PORTC2))
#define GATE_DRIVE_HIGH         (DDRC |= (1<<PORTC2))
#define GATE_DRIVE_LOW          (PORTC &= ~(1<<PORTC2)); (DDRC |= (1<<PORTC2))

#define PITCH_POT 0
#define VOLUME_POT 1
#define WAVE_SELECT_POT 7
#define REGISTER_SELECT_POT 6
#define HYST_SCALE 0.7  // between 0.3 and 0.7 usually
static const int16_t pot_register_selection_hysteresis = 1024.0 / 3 * HYST_SCALE;
static const int16_t pot_waveform_selection_hysteresis = 1024.0 / num_wavetables * HYST_SCALE;

#define MAX_VOLUME 4095         // 12 bit DAC max clamping value

static int16_t pitchDAC = 0;
static int16_t volumeDAC = 0;

static int32_t pitchCalibrationBase = 0;
static int32_t volCalibrationBase = 0;
const int16_t CalibrationTolerance = 15;
const int16_t PitchFreqOffset = 700;
const int16_t VolumeFreqOffset = 700;

#define FREQ_FACTOR 16000000U
#define PITCH_VO_TARGET_FREQUENCY  500000
#define VOLUME_VO_TARGET_FREQUENCY 460800
static float qMeasurement = 0;


Application::Application() {};

void Application::setup() {
    #if SERIAL_MODE == SERIAL_MODE_DEBUG
                Serial.begin(57600);
        #elif SERIAL_MODE == SERIAL_MODE_MIDI
                Serial.begin(31250);
        #endif

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);

    HW_LED_RED_ON; HW_LED_BLUE_OFF; // standby mode

    SPImcpDACinit();
    EEPROM.get(0, pitchDAC);
    EEPROM.get(2, volumeDAC);

    SPImcpDAC2Asend(pitchDAC);
    SPImcpDAC2Bsend(volumeDAC);

    ihInitialiseTimer();
    ihInitialiseInterrupts();

    EEPROM.get(4, pitchCalibrationBase);
    EEPROM.get(8, volCalibrationBase);

    DEBUG_PRINT("P_CAL=");DEBUG_PRINT(pitchCalibrationBase);
    DEBUG_PRINT(" V_CAL=");DEBUG_PRINTLN(volCalibrationBase);
    DEBUG_PRINTLN("Hello, Theremin world!");
}

void Application::loop() {
    int32_t pitch_v = 0, pitch_l = 0; // Last value of pitch    (for filtering)
    int32_t vol_v = 0, vol_l = 0;        // Last value of volume (for filtering and for tracking)

    uint16_t volumePotValue = 0;
    uint16_t pitchPotValue = 0;
    int registerPotValue, registerPotValueL = 0;
    int wavePotValue, wavePotValueL = 0;
    uint8_t registerValue = 2; // octave
    uint16_t tmpVolume;
    uint16_t tmpPitch;

mloop: // Main loop avoiding the GCC "optimization"

    pitchPotValue = analogRead(PITCH_POT);
    volumePotValue = analogRead(VOLUME_POT);
    registerPotValue = analogRead(REGISTER_SELECT_POT);
    wavePotValue = analogRead(WAVE_SELECT_POT);

    if ((registerPotValue - registerPotValueL) >= pot_register_selection_hysteresis ||
        (registerPotValueL - registerPotValue) >= pot_register_selection_hysteresis) { 
        registerPotValueL = registerPotValue;
        // register pot configuration:
        // Left = -1 octave, Center = +/- 0, Right = +1 octave
        if (registerPotValue > 681) {
            registerValue = 1;
            DEBUG_PRINT("REG=");DEBUG_PRINTLN(registerValue);
        } else if (registerPotValue < 342) {
            registerValue = 3;
            DEBUG_PRINT("REG=");DEBUG_PRINTLN(registerValue);
        } else {
            registerValue = 2;
            DEBUG_PRINT("REG=");DEBUG_PRINTLN(registerValue);
        }
    }

    if (((wavePotValue - wavePotValueL) >= pot_waveform_selection_hysteresis) ||
        ((wavePotValueL - wavePotValue) >= pot_waveform_selection_hysteresis)) {
        wavePotValueL = wavePotValue;
        // map 0–1023 to 0–(num_wavetables - 1)
        uint16_t scaled = ((uint32_t)wavePotValue * num_wavetables) / 1024;
        if (scaled >= num_wavetables) scaled = num_wavetables - 1;  // extra safety
        vWavetableSelector = scaled;
        DEBUG_PRINT("WAV=");DEBUG_PRINTLN(vWavetableSelector);
    }

    if (_state == PLAYING && HW_BUTTON_PRESSED) {
        resetTimer();
        _state = CALIBRATING;
        _mode = _mode == NORMAL ? MUTE : NORMAL;
        if (_mode == NORMAL) {
            HW_LED_BLUE_ON; HW_LED_RED_OFF;

            #if INTERPOLATION_MODE == INTERPOLATION_MODE_INTERACTIVE
                if (interpolation_mode == INTERPOLATION_CATMULL_ROM) { interpolation_mode = INTERPOLATION_NONE; }
                else { interpolation_mode++; }
                switch (interpolation_mode) {
                    case INTERPOLATION_NONE: DEBUG_PRINTLN("INT_NONE"); break;
                    case INTERPOLATION_LINEAR: DEBUG_PRINTLN("INT_LINEAR"); break;
                    case INTERPOLATION_HERMITE: DEBUG_PRINTLN("INT_HERMITE"); break;
                    case INTERPOLATION_CATMULL_ROM: DEBUG_PRINTLN("INT_CATMULL"); break;
                    default: DEBUG_PRINTLN("ERR INTERP MODE"); break;
                }
            #endif
        } else {
            HW_LED_BLUE_OFF; HW_LED_RED_ON;
        }
    }

    if (_state == CALIBRATING && HW_BUTTON_RELEASED) {
        _state = PLAYING;
    }

    if (_state == CALIBRATING && timerExpired(65000)) {
        HW_LED_BLUE_ON; HW_LED_RED_ON;

        #ifdef ENABLE_AUDIO_FEEDBACK
        playNote(MIDDLE_C, 150, 25);
        playNote(MIDDLE_C * 2, 150, 25);
        playNote(MIDDLE_C * 4, 150, 25);
        #endif

        bool success = calibrate_pitch();
        if (success) {
            success = calibrate_volume();
        }
        ihInitialiseTimer();
        ihInitialiseInterrupts();

        if (success) {
            calibrate();
            #ifdef ENABLE_AUDIO_FEEDBACK
            playNote(MIDDLE_C * 2, 150, 25);
            playNote(MIDDLE_C * 2, 150, 25);
            #endif
        } else {
            #ifdef ENABLE_AUDIO_FEEDBACK
            playNote(MIDDLE_C * 4, 150, 25);
            playNote(MIDDLE_C, 150, 25);
            #endif
        }

        _mode = NORMAL;
        HW_LED_RED_OFF;

        while (HW_BUTTON_PRESSED) {}
        _state = PLAYING;
    }

    if (pitchValueAvailable) {
        // Averaging pitch values
        pitch_p = pitch;
        pitch_v = pitch;
        pitch_v = pitch_l + ((pitch_v - pitch_l) >> 2); ///< (low-pass) exponential moving average (EMA) approximation
        pitch_l = pitch_v;

        // set wave frequency for each mode
        if (_mode == NORMAL) {
            // 16383 = 2^14 - 1
            // This value is shifted right by registerValue, or the selected octave (or pitch register).
            // If registerValue == 0, no shift: vPointerIncrement = 16383 (maximum playback speed).
            // vPointerIncrement is directly used to advance the phase pointer per sample.
            // Larger values -> faster wavetable traversal -> higher pitch.
            // Using a limit of 16383 ensures that:
            // - The shifted result stays within a reasonable frequency range, avoiding unplayably high tones.
            // - avoid wraparound in the phase accumulator or numeric overflow in later math.
            // - It fits well with common 10.6 fixed-point formats.
            int32_t calcPitch = ((int32_t)pitchCalibrationBase - pitch_v) + 2048 - (pitchPotValue << 2);
            if (calcPitch < 0) { calcPitch = 0; } else if (calcPitch > 16383) { calcPitch = 16383; }
            tmpPitch = (uint16_t)calcPitch;
            setWavetableSampleAdvance(tmpPitch >> registerValue);
            #if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
                if (tmpPitch != pitch_p) { 
                    // output new pitch CV value only if pitch value changed (saves runtime resources)
                    pitch_p = tmpPitch;
                    #if CV_OUTPUT_MODE == CV_OUTPUT_MODE_LOG
                        log_freq = log2U16((uint16_t)tmpPitch);
                        if (log_freq >= 37104) {
                            // 37104 = log2U16(512) + 48*4096/819
                            pitchCV = (int16_t)((819 * (log_freq - 37104)) >> 12);
                            pitchCV >>= registerValue - 1;
                        } else {
                            pitchCV = 0;
                        }
                    #elif CV_OUTPUT_MODE == CV_OUTPUT_MODE_LINEAR
                        // 819Hz/V for Korg & Yamaha
                        pitchCV = tmpPitch >> 2 >> (registerValue - 1);
                    #endif
                    
                    pitchCVAvailable = true;
                }
            #endif
        }

        #if SERIAL_MODE == SERIAL_MODE_MIDI
        if (timerExpired(TICKS_100_MILLIS)) {
            resetTimer();
            Serial.write(pitch & 0xff);
            Serial.write((pitch >> 8) & 0xff);
        }
        #endif
        pitchValueAvailable = false;
    }

    if (volumeValueAvailable && (vol != vol_p)) {
        // Averaging volume values
        vol_p = vol;
        vol = max(vol, 5000);
        vol_v = vol;
        vol_v = vol_l + ((vol_v - vol_l) >> 2); ///< (low-pass) exponential moving average (EMA) approximation
        vol_l = vol_v;

        if (_mode == MUTE) {
            vol_v = 0;
        } else {
            vol_v = MAX_VOLUME - (volCalibrationBase - vol_v) / 2 + (volumePotValue << 2) - 1024;
        }

        // Limit and set volume value
        vol_v = min(vol_v, 4095);
        vol_v = max(vol_v, 0);
        tmpVolume = vol_v >> 4;
        // Give vScaledVolume a pseudo-exponential characteristic:
        vScaledVolume = tmpVolume * (tmpVolume + 2);

        #if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
            // Most synthesizers "exponentiate" the volume CV themselves, thus send the "raw" volume for CV:
            volCV = vol_v;
            volumeCVAvailable = true;
            tmpVolume = tmpVolume >> 1;

            if (!gate_p && (tmpVolume >= GATE_ON)) {
                gate_p = true;
                // pull the gate up to sense, first (to prevent short-circuiting the IO pin:
                GATE_PULLUP;
                if (GATE_SENSE) { 
                    // if it goes up, drive the gate high
                    GATE_DRIVE_HIGH;
                }
            } else if (gate_p && (tmpVolume <= GATE_OFF)) {
                gate_p = false;
                // drive the gate lo
                GATE_DRIVE_LOW;
            }
        #endif
        volumeValueAvailable = false;
    }

    goto mloop; // End of main loop
}


/**
 * @brief Measures the frequency of the SAMPLE_CLK signal (from Q9 of 74HC4060).
 *
 * This function uses Timer1 on the ATmega328P to measure how many clock ticks (at 16 MHz)
 * occur during 31,250 rising edges of the SAMPLE_CLK signal on digital pin 3 (PD3).
 * This corresponds to one full second if SAMPLE_CLK is exactly 31.25 kHz,
 * as expected when Q9 = 16 MHz / 512.
 * 
 * This function acts as a precise reference clock measurement utility.
 * If frequency == 16000000, the SAMPLE_CLK is spot-on at 31.25 kHz.
 * You could normalize this to Hz by dividing 16,000,000 by the result
 * or compare it against expected values for calibration purposes.
 *
 * SAMPLE_CLK is the main wave output interrupt source and this measurement
 * allows precise runtime verification of the generated clock.
 *
 * - Timer1 is used with no prescaling for maximum resolution (ticks in 62.5 ns steps).
 * - Timer1 counter (TCNT1) and overflow counter are combined to form a 32-bit tick count.
 * - This function is blocking and should not be used in real-time critical paths.
 *
 * @note Digital pin 3 (PD3) must be connected to the SAMPLE_CLK signal.
 * @return Total tick count at 16 MHz after 31,250 rising edges — used to compute real frequency.
 */
unsigned long Application::GetQMeasurement() {
    int qn = 0;
    TCCR1B = (1 << CS10);                   ///< Configure Timer1 to run without prescaling (full 16 MHz clock)
    while (!(PIND & (1 << PORTD3))) {}      ///< Wait for a rising edge: current LOW -> wait for HIGH
    while ((PIND & (1 << PORTD3))) {}       ///< Wait for falling edge: current HIGH -> wait for LOW
    // Reset Timer1 and overflow counter
    TCNT1 = 0;
    timer_overflow_counter = 0;
    // Count 31,250 complete cycles of the SAMPLE_CLK signal (1 second at 31.25 kHz)
    while (qn < 31250) {
        while (!(PIND & (1 << PORTD3))) {}  ///< Wait for rising edge
        qn++;
        while ((PIND & (1 << PORTD3))) {}   ///< Wait for falling edge
    }
    TCCR1B = 0;             ///< Stop Timer1 (no clock source)
    // Combine TCNT1 and overflow counter into 32-bit result
    unsigned long frequency = TCNT1;
    unsigned long temp = 65536 * (unsigned long)timer_overflow_counter;
    frequency += temp;
    return frequency;       ///< Number of 16 MHz ticks in 1 second -> directly equals Hz
}

/**
 * @brief Measures the absolute frequency of the pitch oscillator (VO_PITCH).
 *
 * This function uses Timer1 in normal mode to count the number of clock cycles
 * over a 1-second window (via `delay(1000)`) to compute the frequency of the
 * signal arriving on PD5 (ICP1).
 *
 * Timer1 is configured with a prescaler of 1024, so its effective counting frequency is:
 *   16,000,000 / 1024 = 15,625 Hz.
 *
 * The function resets the counter and overflow register before starting measurement,
 * then accumulates the total number of clock ticks over the 1-second duration.
 *
 * @details
 * - This method is typically called during the calibration procedure.
 * - It indirectly measures VO_PITCH, a ~500 kHz RF signal, by leveraging
 *   frequency aliasing from the physical RF circuitry and flip-flop gating (F_PITCH).
 * - Timer1 overflows at 65536 ticks, so the `timer_overflow_counter` accounts for overflows.
 *
 * The final frequency count is:
 * @code
 *   frequency = TCNT1 + 65536 * timer_overflow_counter;
 * @endcode
 *
 * @note The `delay(1000)` does block the MCU — suitable only during setup/calibration,
 * not real-time operation.
 *
 * @return The measured pitch frequency in Timer1 ticks over 1 second, to be converted externally.
 */
unsigned long Application::GetPitchMeasurement() {
    TCNT1 = 0;                      ///< Reset Timer1 counter
    timer_overflow_counter = 0;     ///< Reset overflow counter
    // Set Timer1 with prescaler 1024 (CS12 | CS11 | CS10)
    // Resulting timer tick: 16 MHz / 1024 = 15.625 kHz
    TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10);
    delay(1000);                    ///< Measure duration: 1000 ms (blocking)
    TCCR1B = 0;                     ///< Stop Timer1 after measurement
    // Combine Timer1 count with overflow count to get full frequency count
    unsigned long frequency = TCNT1;
    unsigned long temp = 65536 * (unsigned long)timer_overflow_counter;
    frequency += temp;
    // measured_Hz = measuredTicks * (16e6 / 1024) / 1.0 for 1s integration time
    return frequency;
}

/**
 * @brief Measures the frequency of the volume antenna oscillator (VO_VOL).
 *
 * This function uses Timer/Counter0 in external clock mode to count rising edges
 * from the VO_VOL signal (connected to T0 / PD4 / Digital Pin 4) over a 1-second
 * gate window determined by Timer1 overflow.
 *
 * Timer1 is preloaded with 49911 to overflow after ~1 second (65536 - 49911 = 15625 ticks @ 16 MHz).
 * Timer0 counts external rising edges from VO_VOL during this gate time.
 * The result is the total number of VO_VOL cycles within one second.
 *
 * @return Frequency in Hz as an unsigned long value.
 *
 * @note Timer0 is 8-bit, so an overflow counter is used to extend its range.
 *       This approach trades resolution for simplicity, which is acceptable for volume
 *       since it does not require the same precision or update rate as pitch.
 *
 * @warning This method assumes Timer0 and Timer1 are not used elsewhere during execution.
 */
unsigned long Application::GetVolumeMeasurement() {
    timer_overflow_counter = 0;                                 ///< Clear overflow tracker for Timer0
    TCNT0 = 0;                                                  ///< Reset Timer0 (pulse counter)
    TCNT1 = 49911;                                              ///< Preload Timer1 so it overflows in 1s (65536 - 49911 = 15625 ticks @ 16 MHz)
    // Set Timer0 to external clock on T0 (PD4), count on rising edge
    TCCR0B = (1 << CS02) | (1 << CS01) | (1 << CS00);
    // Clear Timer1 overflow flag (to detect overflow event)
    TIFR1 = (1 << TOV1);                                        //Timer1 INT Flag Reg: Clear Timer Overflow Flag
    while (!(TIFR1 & ((1 << TOV1)))) {}                         // Wait until Timer1 overflows (1s interval)
    TCCR0B = 0;                                                 // Stop TimerCounter 0
    unsigned long frequency = TCNT0;                            // Retrieve measured pulse count from Timer0 and overflow counter
    unsigned long temp = (unsigned long)timer_overflow_counter; // Each Timer0 overflow = 256 pulses
    frequency += temp * 256;
    return frequency;
}

/**
 * @brief Runs the automatic calibration routine for pitch and volume oscillators.
 *
 * This function captures reference frequency measurements from both VO_PITCH and VO_VOL,
 * storing baseline counter values and computing calibration constants used during runtime
 * for frequency-to-pitch/volume translation. The results are saved into EEPROM for persistence.
 *
 * ### Calibration Steps:
 * 1. **Pitch:**
 *    - Resets internal pitch flags and timers.
 *    - Waits for a valid `pitchValueAvailable` flag, or times out after 10 ms.
 *    - Captures the base pitch period and computes:
 *      - `pitchCalibrationBase`: raw Timer1 delta
 *
 * 2. **Volume:**
 *    - Same logic as pitch, using `volValueAvailable` and `vol_counter`.
 *    - Captures `volCalibrationBase` from the Timer1 delta.
 *
 * 3. **EEPROM Storage:**
 *    - Stores calibration values:
 *      - At address 4: `pitchCalibrationBase` (4 bytes)
 *      - At address 8: `volCalibrationBase` (4 bytes)
 *
 * @note Calibration uses blocking wait with optional timeout of 10 ms for each channel.
 *       It assumes the player is holding still, and RF conditions are stable.
 *
 * @warning Requires successful pitch/volume ISR processing to populate
 *          `pitchValueAvailable` and `volumeValueAvailable` before timeout.
 *
 * @see Application::GetPitchMeasurement()
 * @see Application::GetVolumeMeasurement()
 */
void Application::calibrate() {
    // --- Pitch calibration ---
    resetPitchFlag();   //< Clear pitchValueAvailable
    resetTimer();       ///< Reset system timer
    savePitchCounter(); ///< Store previous pitch counter value
    // Wait for new pitch value or timeout after 10ms
    while (!pitchValueAvailable && timerUnexpiredMillis(10)) {}
    pitchCalibrationBase = pitch;           ///< Store raw pitch frequency

    // --- Volume calibration ---
    resetVolFlag();     ///< Clear volumeValueAvailable
    resetTimer();       ///< Reset system timer
    saveVolCounter();   ///< Store previous volume counter value
    // Wait for new volume value or timeout after 10ms
    while (!volumeValueAvailable && timerUnexpiredMillis(10)) {}
    volCalibrationBase = vol;               ///< Store raw volume frequency

    // --- Store calibration results in EEPROM ---
    EEPROM.put(4, pitchCalibrationBase);    ///< Save pitch baseline (4 bytes)
    EEPROM.put(8, volCalibrationBase);      ///< Save volume baseline (4 bytes)

    DEBUG_PRINT("P CAL=");DEBUG_PRINT(pitchCalibrationBase);
    DEBUG_PRINT(" V CAL=");DEBUG_PRINTLN(volCalibrationBase);
}

/**
 * @brief Performs pitch oscillator calibration using a numerical approximation.
 *
 * This function tunes the DAC output controlling the VO_PITCH oscillator to align
 * the oscillator frequency with the reference frequency derived from the 16 MHz crystal.
 *
 * ### Calibration Goal:
 * - Find the correct DAC setting (`pitchXn`) so that the pitch oscillator outputs exactly 500 kHz,
 *   matching the divided reference frequency (U2 Q5 = 16 MHz / 32).
 *
 * ### Algorithm Summary:
 * - The search interval is initialized with `pitchXn0 = 0` and `pitchXn1 = 4095`.
 * - It uses a form of secant method (similar to dichotomy/bisection) to iteratively find
 *   the optimal DAC value that minimizes the frequency delta (`pitchfn0 - pitchfn1`).
 * - Loop is limited to 12 iterations, sufficient for 12-bit resolution convergence.
 *
 * ### EEPROM Storage:
 * - The final calibrated DAC value is written to EEPROM address 0.
 *
 * @warning Requires VO_PITCH oscillator to be stable during calibration.
 * @note Uses blocking `delay()` and relies on GetPitchMeasurement(), which uses Timer1 counting.
 * @see Application::GetQMeasurement()
 * @see Application::GetPitchMeasurement()
 */
bool Application::calibrate_pitch() {
    static int16_t pitchXn0 = 0;    ///< Lower DAC bound
    static int16_t pitchXn1 = 0;    ///< Upper DAC bound (max for 12-bit DAC)
    static int16_t pitchXn2 = 0;    ///< Midpoint in iteration
    static float q0 = 0;            ///< Calculated nominal frequency based on measured X1
    static long pitchfn0 = 0;       ///< Measured pitch frequencies
    static long pitchfn1 = 0;
    static long pitchfn = 0;

    DEBUG_PRINTLN("\nCAL PITCH");

    HW_LED_BLUE_ON; HW_LED_RED_ON;

    ihInitialisePitchMeasurement();     ///< Setup Timer1 and associated ISR flags
    interrupts();
    SPImcpDACinit();                    ///< Initialize DACs

    qMeasurement = GetQMeasurement(); // Measure Arudino clock frequency
    DEBUG_PRINT("q="); DEBUG_PRINTLN(qMeasurement);
    // Measure effective X1 quartz frequency via Q9 (SAMPLE_CLK) at 31.25 kHz
    q0 = (FREQ_FACTOR / qMeasurement * PITCH_VO_TARGET_FREQUENCY); ///< Count Timer1 ticks over 31.25k cycles

    pitchXn0 = 0;
    pitchXn1 = 4095;

    pitchfn = q0 - PitchFreqOffset; ///< Correct for calibration drift

    DEBUG_PRINT("Set f= "); DEBUG_PRINTLN(pitchfn);

    SPImcpDAC2Bsend(1600);          ///< bias the volume oscillator for pitch calibration
    SPImcpDAC2Asend(pitchXn0);      ///< Frequency at low DAC value
    delay(100);
    pitchfn0 = GetPitchMeasurement();
    SPImcpDAC2Asend(pitchXn1);      ///< Frequency at high DAC value
    delay(100);
    pitchfn1 = GetPitchMeasurement();

    DEBUG_PRINT("Tune range "); DEBUG_PRINT(pitchfn0); DEBUG_PRINT(" to "); DEBUG_PRINTLN(pitchfn1);

    int8_t cal_iteration = 0; ///< Max 12 iterations
    while (abs(pitchfn0 - pitchfn1) > CalibrationTolerance && (cal_iteration < 12)) {
        SPImcpDAC2Asend(pitchXn0);
        delay(100);
        pitchfn0 = GetPitchMeasurement() - pitchfn;
        SPImcpDAC2Asend(pitchXn1);
        delay(100);
        pitchfn1 = GetPitchMeasurement() - pitchfn;
        // Secant approximation to refine DAC value
        pitchXn2 = pitchXn1 - ((pitchXn1 - pitchXn0) * pitchfn1) / (pitchfn1 - pitchfn0); // new DAC value

        DEBUG_PRINT("\nDAC L: "); DEBUG_PRINT(pitchXn0); DEBUG_PRINT(" fL: "); DEBUG_PRINTLN(pitchfn0);
        DEBUG_PRINT("DAC H: "); DEBUG_PRINT(pitchXn1); DEBUG_PRINT(" fH: "); DEBUG_PRINTLN(pitchfn1);

        pitchXn0 = pitchXn1;
        pitchXn1 = pitchXn2;

        HW_LED_BLUE_TOGGLE;     ///< visual feedback
        cal_iteration++;
    }
    delay(100);
    bool success = cal_iteration < 12;
    if (success) {
        EEPROM.put(0, pitchXn0); ///< Store final calibrated DAC value for VO_PITCH
    }
    return success;
}

/**
 * @brief Performs volume oscillator calibration using iterative frequency measurement.
 *
 * This function finds the optimal DAC value that aligns the VO_VOL oscillator frequency
 * with the reference frequency derived from the second crystal (X2 at 7.3728 MHz).
 *
 * ### Calibration Goal:
 * - Align the VO_VOL oscillator frequency with a nominal target:  
 *   7.3728 MHz / 16 (Q4 from U2) = 460,800 Hz (rounded here to 460,756 Hz).
 *
 * ### Calibration Strategy:
 * - Like the pitch routine, this uses a secant-like numerical approach over 12-bit DAC range.
 * - Uses a 100 ms delay (`waitNOP(44316)`) between DAC value changes and frequency measurement
 *   to ensure the RF circuit stabilizes.
 *
 * ### EEPROM Storage:
 * - The resulting DAC value is stored at EEPROM address 2.
 *
 * @note The DAC2B channel is used for volume oscillator control.
 * @warning Requires stable oscillator and clean signal edges on VO_VOL for accurate timing.
 * @see Application::GetVolumeMeasurement()
 */
bool Application::calibrate_volume() {
    static int16_t volumeXn0 = 0;   ///< Lower DAC bound
    static int16_t volumeXn1 = 0;   ///< Upper DAC bound
    static int16_t volumeXn2 = 0;   ///< Next DAC trial value
    static float q0 = 0;            ///< Calculated oscillator frequency based on crystal measurement
    static long volumefn0 = 0;      ///< Frequency measurements
    static long volumefn1 = 0;
    static long volumefn = 0;

    DEBUG_PRINTLN("\nCAL VOL");

    ihInitialiseVolumeMeasurement();    //< Configure Timer0 and related registers
    interrupts();
    SPImcpDACinit();                    ///< Re-initialize DAC interface

    volumeXn0 = 0;
    volumeXn1 = 4095;

    // Calculate actual frequency from X2 crystal via prior qMeasurement (from X1 at 16 MHz)
    // Target frequency: 7372800 / 16 = 460800
    q0 = (FREQ_FACTOR / qMeasurement * VOLUME_VO_TARGET_FREQUENCY);
    volumefn = q0 - VolumeFreqOffset;   ///< Subtract drift offset (700 Hz by default)

    DEBUG_PRINT("Vf="); DEBUG_PRINTLN(volumefn);

    // Measure low and high frequency extremes for DAC sweep range
    SPImcpDAC2Bsend(volumeXn0);
    delay_NOP(44316); //44316 ≈ 100 ms
    volumefn0 = GetVolumeMeasurement();
    SPImcpDAC2Bsend(volumeXn1);
    delay_NOP(44316); //44316 ≈ 100 ms
    volumefn1 = GetVolumeMeasurement();

    DEBUG_PRINT("Tune range "); DEBUG_PRINT(volumefn0); DEBUG_PRINT(" to "); DEBUG_PRINTLN(volumefn1);

    int8_t cal_iteration = 0; ///< Max 12 iterations
    while (abs(volumefn0 - volumefn1) > CalibrationTolerance && (cal_iteration < 12)) {
        SPImcpDAC2Bsend(volumeXn0);
        delay_NOP(44316); //44316=100ms
        volumefn0 = GetVolumeMeasurement() - volumefn;
        SPImcpDAC2Bsend(volumeXn1);
        delay_NOP(44316); //44316=100ms
        volumefn1 = GetVolumeMeasurement() - volumefn;
        // Secant method refinement
        volumeXn2 = volumeXn1 - ((volumeXn1 - volumeXn0) * volumefn1) / (volumefn1 - volumefn0); // calculate new DAC value

        DEBUG_PRINT("\nDAC L: "); DEBUG_PRINT(volumeXn0); DEBUG_PRINT(" fL: "); DEBUG_PRINTLN(volumefn0);
        DEBUG_PRINT("DAC H: "); DEBUG_PRINT(volumeXn1); DEBUG_PRINT(" fH: "); DEBUG_PRINTLN(volumefn1);

        volumeXn0 = volumeXn1;
        volumeXn1 = volumeXn2;

        HW_LED_BLUE_TOGGLE;     ///< visual feedback
        cal_iteration++;
    }
    bool success = cal_iteration < 12;
    if (success) {
        EEPROM.put(2, volumeXn0); ///< Store DAC value for volume oscillator compensation
    }
    HW_LED_BLUE_ON; HW_LED_RED_OFF;
    return success;
}

#ifdef ENABLE_AUDIO_FEEDBACK
void Application::playNote(float hz, uint16_t milliseconds = 500, uint8_t volume = 255) {
    vScaledVolume = volume * (volume + 2);
    setWavetableSampleAdvance((uint16_t)(hz * HZ_ADDVAL_FACTOR));
    millitimer(milliseconds);
    vScaledVolume = 0;
}
#endif

void Application::delay_NOP(unsigned long time) {
    volatile unsigned long i = 0;
    for (i = 0; i < time; i++) {
        __asm__ __volatile__("nop");
    }
}

#if CV_OUTPUT_MODE == CV_OUTPUT_MODE_LOG
// calculate log2 of an unsigned from 1 to 65535 into a 4.12 fixed point unsigned
// To avoid use of log (double) function
// Fixed-point log2 approximation: input is uint16_t linear value
// Output is 4.12 unsigned fixed-point result (suitable for amplitude scaling, etc.)

#define FIXED_1      (1UL << 16)    // 16.16 fixed-point "1.0"
#define FIXED_HALF   (1UL << 15)
#define LOG_SCALE    12
#define POLY_SHIFT   15
#define OUTPUT_SHIFT 3
#define BASE_1_0     32768          // 1.0 in s15 fixed-point

// Polynomial coefficients for log2(x) where x ∈ [1.0, 2.0]
#define POLY_A0  37
#define POLY_A1  46390
#define POLY_A2 -18778
#define POLY_A3   5155

uint16_t Application::log2U16(uint16_t lin_input) {
    if (lin_input == 0)
        return 0;

    // Convert input to 16.16 fixed-point
    uint32_t long_lin = ((uint32_t)lin_input) << 16;
    uint32_t log_output = 0;

    // Fast bit-shifting to isolate integer part of log2
    if (long_lin >= (256UL << 16)) {  // 2^8
        log_output += (8 << LOG_SCALE);
        long_lin >>= 8;
    }
    if (long_lin >= (16UL << 16)) {   // 2^4
        log_output += (4 << LOG_SCALE);
        long_lin >>= 4;
    }
    if (long_lin >= (4UL << 16)) {    // 2^2
        log_output += (2 << LOG_SCALE);
        long_lin >>= 2;
    }
    if (long_lin >= (2UL << 16)) {    // 2^1
        log_output += (1 << LOG_SCALE);
        long_lin >>= 1;
    }

    // Now long_lin ∈ [1.0, 2.0) in 16.16 -> reduce to 17.15 for signed math
    long_lin >>= 1;

    int32_t x = (int32_t)long_lin - BASE_1_0; // x = (x - 1) in s15
    int32_t x2 = ((int64_t)x * x) >> POLY_SHIFT;
    int32_t x3 = ((int64_t)x2 * x) >> POLY_SHIFT;

    // Polynomial approximation: log2(x) ≈ A0 + A1·x + A2·x^2 + A3·x^3
    int32_t poly = POLY_A0
                 + (((int64_t)POLY_A1 * x) >> POLY_SHIFT)
                 + (((int64_t)POLY_A2 * x2) >> POLY_SHIFT)
                 + (((int64_t)POLY_A3 * x3) >> POLY_SHIFT);

    log_output += (poly >> OUTPUT_SHIFT);  // Adjust to 4.12 fixed-point

    return (uint16_t)log_output;
}
#endif