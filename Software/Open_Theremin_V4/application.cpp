#include "application.h"

#include "hw.h"
#include "SPImcpDAC.h"
#include "ihandlers.h"
#include "timer.h"
#include "EEPROM.h"

const int16_t CalibrationTolerance = 15;
const int16_t PitchFreqOffset = 700;
const int16_t VolumeFreqOffset = 700;
const int8_t HYST_VAL = 140;

static int32_t pitchCalibrationBase = 0;
static int16_t pitchDAC = 0;
static int16_t volumeDAC = 0;
static float qMeasurement = 0;

static int32_t volCalibrationBase = 0;

// UI parameters (touch button and potentiometers)
#define UI_BUTTON_LONG_PRESS_DURATION   60000
// Potentiometer variables, hysteresis and scaling

#define HYST_SCALE 0.95
static const int16_t pot_register_selection_hysteresis = 1024.0 / 3 * HYST_SCALE;   // only three position octave selection
static const int16_t pot_waveform_selection_hysteresis = 1024.0 / num_wavetables * HYST_SCALE;  // map the waveform selection poti depending on how many waveforms are being loaded in the DDS generator
static const int16_t pot_rf_virtual_field_adjust_hysteresis = 1024 / 64;    // reduce the volume and pitch field potis to 64 steps to slightly reduce audio jitter when moving them
int16_t pitchPotValue = 0, pitchPotValueL = 0;
int16_t volumePotValue = 0, volumePotValueL = 0;
int16_t registerPotValue = 0, registerPotValueL = 0;
int16_t wavePotValue = 0, wavePotValueL = 0;
uint8_t registerValue = 2; // octave register
uint8_t registerValueL = 2; // octave register compare

/**
 * @brief Read all the potentiometer position and update the changed values
 *        if the hysteresis constant have been surpassed.
 * 
 * @param force
 *        optional boolean flag to force the updates (at power-on)
 */
void ui_potis_read_all(bool force = false) {
    pitchPotValueL = analogRead(PITCH_POT);
    if (force || abs(pitchPotValue - pitchPotValueL) >= pot_rf_virtual_field_adjust_hysteresis) { 
        pitchPotValue = pitchPotValueL;
    }
    
    volumePotValueL = analogRead(VOLUME_POT);
    if (force || abs(volumePotValue - volumePotValueL) >= pot_rf_virtual_field_adjust_hysteresis) { 
        volumePotValue = volumePotValueL;
    }

    registerPotValueL = analogRead(REGISTER_SELECT_POT);
    if (force || abs(registerPotValue - registerPotValueL) >= pot_register_selection_hysteresis) { 
        registerPotValue = registerPotValueL;
        // register pot offset configuration:
        // Left = -1 octave, Center = 0, Right = +1 octave
        if (registerPotValue > pot_register_selection_hysteresis * 2) {
            registerValueL = 1;
            DEBUG_PRINTLN(F("OCT+1"));
        } else if (registerPotValue < pot_register_selection_hysteresis) {
            registerValueL = 3;
            DEBUG_PRINTLN(F("OCT-1"));
        } else {
            registerValueL = 2;
            DEBUG_PRINTLN(F("OCT+0"));
        }
        if (registerValueL != registerValue) {
            registerValue = registerValueL;
        }
    }

    wavePotValueL = analogRead(WAVE_SELECT_POT);
    if (force || abs(wavePotValue - wavePotValueL) >= pot_waveform_selection_hysteresis) {
        wavePotValue = wavePotValueL;
        // map 0–1023 to 0–(num_wavetables - 1)
        uint16_t scaled = ((uint32_t)wavePotValue * num_wavetables) / 1024;
        if (scaled >= num_wavetables) scaled = num_wavetables - 1;    // extra safety
        if (scaled != vWavetableSelector) {
            vWavetableSelector = scaled;
            DEBUG_PRINT(F("WAV="));DEBUG_PRINTLN(scaled);
        }
    }
}


Application::Application() {}

void Application::setup() {
    // initializes the serial port if needed to the correct speed
    // see "build.h" file
#if SERIAL_PORT_MODE & SERIAL_PORT_MODE_MIDI
    Serial.begin(SERIAL_SPEED_MIDI);
#elif SERIAL_PORT_MODE == SERIAL_PORT_MODE_DEBUG
    Serial.begin(SERIAL_SPEED_DEBUG);
#endif

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);

    // red LED indicates standby (muted)
    // blue LED is lit during performance mode.
    HW_LED_BLUE_OFF; HW_LED_RED_ON;

    // initialize potentiometer position at startup
    // force update for correct initialization
    ui_potis_read_all(true);

    // read complete configuration parameters before initialization
    EEPROM.get(EEPROM_PITCH_DAC_VOLTAGE_ADDRESS, pitchDAC);
    EEPROM.get(EEPROM_VOLUME_DAC_VOLTAGE_ADDRESS, volumeDAC);
    EEPROM.get(EEPROM_PITCH_DAC_CALIBRATION_BASE_ADDRESS, pitchCalibrationBase);
    EEPROM.get(EEPROM_VOLUME_DAC_CALIBRATION_BASE_ADDRESS, volCalibrationBase);
    // use EEPROM values for calibration
    SPImcpDACinit();
    SPImcpDAC2Asend(pitchDAC);
    SPImcpDAC2Bsend(volumeDAC);

    // start the DDS wave generator
    ihInitialiseTimer();
    ihInitialiseInterrupts();
}

unsigned long Application::GetQMeasurement() {
    TCCR1B = (1 << CS10);
    while (!(PIND & (1 << PORTD3))) {}
    while ((PIND & (1 << PORTD3))) {}

    TCNT1 = 0;
    timer_overflow_counter = 0;
    int qn = 0;
    while (qn < 31250) {
        while (!(PIND & (1 << PORTD3))) {}
        qn++;
        while ((PIND & (1 << PORTD3))) {}
    }
    TCCR1B = 0;

    unsigned long frequency = TCNT1;
    unsigned long temp = 65536 * (unsigned long)timer_overflow_counter;
    frequency += temp;
    return frequency;
}

unsigned long Application::GetPitchMeasurement() {
    TCNT1 = 0;
    timer_overflow_counter = 0;
    TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10);
    delay(1000);
    TCCR1B = 0;

    unsigned long frequency = TCNT1;
    unsigned long temp = 65536 * (unsigned long)timer_overflow_counter;
    frequency += temp;
    return frequency;
}

unsigned long Application::GetVolumeMeasurement() {
    timer_overflow_counter = 0;

    TCNT0 = 0;
    TCNT1 = 49911;
    TCCR0B = (1 << CS02) | (1 << CS01) | (1 << CS00);   // External clock source on T0 pin. Clock on rising edge.
    TIFR1 = (1 << TOV1);                                // Timer1 INT Flag Reg: Clear Timer Overflow Flag

    while (!(TIFR1 & ((1 << TOV1)))) {}
    // on Timer 1 overflow (1s)
    TCCR0B = 0;                                                 // Stop TimerCounter 0
    unsigned long frequency = TCNT0;                            // get counter 0 value
    unsigned long temp = (unsigned long)timer_overflow_counter; // and overflow counter
    frequency += temp * 256;
    return frequency;
}

void Application::loop() {
    int32_t pitch_v = 0, pitch_l = 0;   // Last value of pitch    (for filtering)
    int32_t vol_v = 0, vol_l = 0;       // Last value of volume (for filtering and for tracking)

    uint16_t tmpVolume;
    int16_t tmpPitch;

mloop: // Main loop avoiding the GCC "optimization"
    // continuosly read potentiometers and flag changed values
    ui_potis_read_all();

    if (_state == PLAYING && HW_BUTTON_PRESSED) {
        resetTimer();
        _state = CALIBRATING;
        _mode = _mode == NORMAL ? MUTE : NORMAL;
        if (_mode == NORMAL) {
            // blue LED indicates performance mode (play)
            HW_LED_BLUE_ON; HW_LED_RED_OFF;
        } else {
            // red LED indicates mute (standby)
            HW_LED_BLUE_OFF; HW_LED_RED_ON;
        }
    }

    if (_state == CALIBRATING && HW_BUTTON_RELEASED) {
        _state = PLAYING;
    }

    if (_state == CALIBRATING && timerExpired(UI_BUTTON_LONG_PRESS_DURATION)) {
        // both LEDs indicate calibration mode
        HW_LED_BLUE_ON; HW_LED_RED_ON;

        playStartupSound();

        calibrate_pitch();
        calibrate_volume();

        ihInitialiseTimer();
        ihInitialiseInterrupts();

        playCalibratingCountdownSound();
        calibrate();

        _mode = NORMAL;
        HW_LED_RED_OFF;

        while (HW_BUTTON_PRESSED) {}
        _state = PLAYING;
    }

#if SERIAL_PORT_MODE == SERIAL_PORT_MODE_MIDI
    // MIDI note events are sent out only if enabled in "build.h"
    if (timerExpired(TICKS_100_MILLIS)) {
        resetTimer();
        Serial.write(pitch & 0xff);
        Serial.write((pitch >> 8) & 0xff);
    }
#endif

    if (pitchValueAvailable) { 
        // If capture event
        pitch_p = pitch;
        pitch_v = pitch; // Averaging pitch values
        pitch_v = pitch_l + ((pitch_v - pitch_l) >> 2);
        pitch_l = pitch_v;

        // set wave frequency for each mode
        if (_mode == NORMAL) {
            // if playing and not mute
            tmpPitch = ((pitchCalibrationBase - pitch_v) + 2048 - (pitchPotValue << 2));
            tmpPitch = min(tmpPitch, 16383);    // Unaudible upper limit just to prevent DAC overflow
            tmpPitch = max(tmpPitch, 0);            // Silence behing zero beat
            setWavetableSampleAdvance(tmpPitch >> registerValue);
            if (tmpPitch != pitch_p) { 
                pitch_p = tmpPitch;
#if CV_OUTPUT_MODE == CV_OUTPUT_MODE_LOG
                    // output new pitch CV value only if pitch value changed (saves runtime resources)
                    // 1V/Oct for Moog & Roland
                    uint16_t tmpOct = 0;
                    while (tmpPitch > 1023) {
                        tmpOct += 819;
                        tmpPitch >>= 1;
                    }
                    tmpPitch -= 512;
                    tmpPitch = max(tmpPitch, 0);
                    uint16_t tmpLog = (((uint32_t)tmpPitch * 819) >> 9);
                    pitchCV = (tmpOct + tmpLog) - 48;
                    pitchCV = max(pitchCV, 0);
                    pitchCVAvailable = true;
#elif CV_OUTPUT_MODE == CV_OUTPUT_MODE_LINEAR
                    // 819Hz/V for Korg & Yamaha
                    pitchCV = tmpPitch >> 2;
                    pitchCVAvailable = true;
#endif // CV OUT IF ENABLED
            }
        }
        pitchValueAvailable = false;
    }

    if (volumeValueAvailable && (vol != vol_p)) { 
        // If capture event AND volume value changed (saves runtime resources)
        vol_p = vol;
        vol = max(vol, 5000);

        vol_v = vol; // Averaging volume values
        vol_v = vol_l + ((vol_v - vol_l) >> 2);
        vol_l = vol_v;

        if (_mode == MUTE) {
            vScaledVolume = 0;
        } else {
            vol_v = MAX_VOLUME - (volCalibrationBase - vol_v) / 2 + (volumePotValue << 2) - 1024;
            // Limit and set volume value
            vol_v = min(vol_v, 4095);
            vol_v = max(vol_v, 0);
            tmpVolume = vol_v >> 4;
            // Give vScaledVolume a pseudo-exponential characteristic for the final audio output
            vScaledVolume = tmpVolume * (tmpVolume + 2);
        }

#if CV_OUTPUT_MODE != CV_OUTPUT_MODE_OFF
        tmpVolume = tmpVolume >> 1;
        // Most synthesizers "exponentiate" the volume CV themselves
        // so we send the "raw" volume for CV
        volCV = vol_v;
        volumeCVAvailable = true;

        // Drive the GATE output pin if CV is activated in "build.h"
        if (!gate_p && (tmpVolume >= GATE_ON_THRESHOLD_LEVEL)) {
            gate_p = true;
            // pull the gate up to sense, first (to prevent short-circuiting the IO pin:
            GATE_PULLUP;
            if (GATE_SENSE) { 
                // if it goes up, drive the gate full high:
                GATE_DRIVE_HIGH;
            }
        } else if (gate_p && (tmpVolume <= GATE_OFF_THRESHOLD_LEVEL)) {
            gate_p = false;
            // drive the gate low:
            GATE_DRIVE_LOW;
        }
#endif // GATE IF ENABLED
        volumeValueAvailable = false;
    }

    goto mloop; // End of main loop
}

void Application::calibrate() {
    pitchValueAvailable = false;
    resetTimer();
    pitch_counter_l = pitch_counter;
    while (!pitchValueAvailable && timerUnexpiredMillis(10)) {}
    pitchCalibrationBase = pitch;

    volumeValueAvailable = false;
    resetTimer();
    vol_counter_l = vol_counter;
    while (!volumeValueAvailable && timerUnexpiredMillis(10)) {}
    volCalibrationBase = vol;

    EEPROM.put(EEPROM_PITCH_DAC_CALIBRATION_BASE_ADDRESS, pitchCalibrationBase);
    EEPROM.put(EEPROM_VOLUME_DAC_CALIBRATION_BASE_ADDRESS, volCalibrationBase);
}

void Application::calibrate_pitch() {
    static int16_t pitchXn0 = 0;
    static int16_t pitchXn1 = 0;
    static int16_t pitchXn2 = 0;
    static float q0 = 0;
    static long pitchfn0 = 0;
    static long pitchfn1 = 0;
    static long pitchfn = 0;

    DEBUG_PRINTLN("\nPITCH CALIBRATION\n");

    HW_LED_BLUE_ON; HW_LED_RED_ON;

    ihInitialisePitchMeasurement();
    interrupts();
    SPImcpDACinit();

    qMeasurement = GetQMeasurement(); // Measure X1 clock frequency
    DEBUG_PRINT("X1 Freq: "); DEBUG_PRINTLN(qMeasurement);

    q0 = (16000000 / qMeasurement * 500000); //Calculated set frequency based on X1 clock frequency

    pitchXn0 = 0;
    pitchXn1 = 4095;

    pitchfn = q0 - PitchFreqOffset; // Add offset calue to set frequency

    DEBUG_PRINT("\nPitch Set Frequency: "); DEBUG_PRINTLN(pitchfn);

    SPImcpDAC2Bsend(1600);

    SPImcpDAC2Asend(pitchXn0);
    delay(100);
    pitchfn0 = GetPitchMeasurement();

    SPImcpDAC2Asend(pitchXn1);
    delay(100);
    pitchfn1 = GetPitchMeasurement();

    DEBUG_PRINT("Frequency tuning range: "); DEBUG_PRINT(pitchfn0); DEBUG_PRINT(" to "); DEBUG_PRINTLN(pitchfn1);

    while (abs(pitchfn0 - pitchfn1) > CalibrationTolerance) {
        SPImcpDAC2Asend(pitchXn0);
        delay(100);
        pitchfn0 = GetPitchMeasurement() - pitchfn;

        SPImcpDAC2Asend(pitchXn1);
        delay(100);
        pitchfn1 = GetPitchMeasurement() - pitchfn;

        pitchXn2 = pitchXn1 - ((pitchXn1 - pitchXn0) * pitchfn1) / (pitchfn1 - pitchfn0); // new DAC value

        DEBUG_PRINT("\nDAC value L: "); DEBUG_PRINT(pitchXn0); DEBUG_PRINT(" Freq L: "); DEBUG_PRINTLN(pitchfn0);
        DEBUG_PRINT("DAC value H: "); DEBUG_PRINT(pitchXn1); DEBUG_PRINT(" Freq H: "); DEBUG_PRINTLN(pitchfn1);

        pitchXn0 = pitchXn1;
        pitchXn1 = pitchXn2;

        HW_LED_BLUE_TOGGLE;
    }
    delay(100);

    EEPROM.put(EEPROM_PITCH_DAC_VOLTAGE_ADDRESS, pitchXn0);
}

void Application::calibrate_volume() {
    static int16_t volumeXn0 = 0;
    static int16_t volumeXn1 = 0;
    static int16_t volumeXn2 = 0;
    static float q0 = 0;
    static long volumefn0 = 0;
    static long volumefn1 = 0;
    static long volumefn = 0;

    DEBUG_PRINTLN("\nVOLUME CALIBRATION");

    ihInitialiseVolumeMeasurement();
    interrupts();
    SPImcpDACinit();

    volumeXn0 = 0;
    volumeXn1 = 4095;

    q0 = (16000000 / qMeasurement * 460765);
    volumefn = q0 - VolumeFreqOffset;

    DEBUG_PRINT("\nVolume Set Frequency: "); DEBUG_PRINTLN(volumefn);

    SPImcpDAC2Bsend(volumeXn0);
    delay_NOP(44316); //44316=100ms

    volumefn0 = GetVolumeMeasurement();

    SPImcpDAC2Bsend(volumeXn1);

    delay_NOP(44316); //44316=100ms
    volumefn1 = GetVolumeMeasurement();

    DEBUG_PRINT("Frequency tuning range: "); DEBUG_PRINT(volumefn0); DEBUG_PRINT(" to "); DEBUG_PRINTLN(volumefn1);

    while (abs(volumefn0 - volumefn1) > CalibrationTolerance) {
        SPImcpDAC2Bsend(volumeXn0);
        delay_NOP(44316); //44316=100ms
        volumefn0 = GetVolumeMeasurement() - volumefn;

        SPImcpDAC2Bsend(volumeXn1);
        delay_NOP(44316); //44316=100ms
        volumefn1 = GetVolumeMeasurement() - volumefn;

        volumeXn2 = volumeXn1 - ((volumeXn1 - volumeXn0) * volumefn1) / (volumefn1 - volumefn0); // calculate new DAC value

        DEBUG_PRINT("\nDAC value L: "); DEBUG_PRINT(volumeXn0); DEBUG_PRINT(" Freq L: "); DEBUG_PRINTLN(volumefn0);
        DEBUG_PRINT("DAC value H: "); DEBUG_PRINT(volumeXn1); DEBUG_PRINT(" Freq H: "); DEBUG_PRINTLN(volumefn1);

        volumeXn0 = volumeXn1;
        volumeXn1 = volumeXn2;
        HW_LED_BLUE_TOGGLE;
    }
    EEPROM.put(EEPROM_VOLUME_DAC_VOLTAGE_ADDRESS, volumeXn0);

    HW_LED_BLUE_ON; HW_LED_RED_OFF;

    DEBUG_PRINTLN("\nCALIBRATION COMPLETED\n");
}

void Application::playNote(float hz, uint16_t milliseconds = 500, uint8_t volume = 255) {
    vScaledVolume = volume * (volume + 2);
    setWavetableSampleAdvance((uint16_t)(hz * HZ_ADDVAL_FACTOR));
    millitimer(milliseconds);
    vScaledVolume = 0;
}

void Application::playStartupSound() {
    playNote(MIDDLE_C, 150, 25);
    playNote(MIDDLE_C * 2, 150, 25);
    playNote(MIDDLE_C * 4, 150, 25);
}

void Application::playCalibratingCountdownSound() {
    playNote(MIDDLE_C * 2, 150, 25);
    playNote(MIDDLE_C * 2, 150, 25);
}

void Application::delay_NOP(unsigned long time) {
    volatile unsigned long i = 0;
    for (i = 0; i < time; i++) {
        __asm__ __volatile__("nop");
    }
}
