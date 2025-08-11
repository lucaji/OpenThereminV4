#ifndef _APPLICATION_H
#define _APPLICATION_H

#include <Arduino.h>

#include "build.h"

enum AppState { CALIBRATING = 0, PLAYING };
enum AppMode { MUTE = 0, NORMAL };

class Application {
    public:
    Application();
    
    void setup();
    void loop();
    
    private:
    static const uint16_t MAX_VOLUME = 4095;

    int16_t pitch_p = -1; 
    int32_t vol_p = -1;
    bool gate_p = false;

    AppState _state = PLAYING;
    AppMode    _mode = MUTE;
        
    void calibrate();
    void calibrate_pitch();
    void calibrate_volume();
    
    unsigned long GetPitchMeasurement();
    unsigned long GetVolumeMeasurement();
    unsigned long GetQMeasurement();

    const float HZ_ADDVAL_FACTOR = 2.09785;
    const float MIDDLE_C = 261.6;

    void playNote(float hz, uint16_t milliseconds, uint8_t volume);
    void playStartupSound();
    void playCalibratingCountdownSound();
    void delay_NOP(unsigned long time);
};

#endif // _APPLICATION_H
