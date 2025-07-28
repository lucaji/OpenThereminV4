#ifndef _APPLICATION_H
#define _APPLICATION_H

#include <Arduino.h>
#include <avr/io.h>

#include "build.h"

enum AppState { CALIBRATING = 0, PLAYING };
enum AppMode  { MUTE = 0, NORMAL };

class Application {
  public:
    Application();
    
    void setup();
    void loop();
    
  private:

    int16_t pitch_p = -1; 
    int32_t vol_p = -1;
    bool gate_p = false;

    AppState _state = PLAYING;
    AppMode  _mode = MUTE;
        
    void calibrate();
    bool calibrate_pitch();
    bool calibrate_volume();
    
    unsigned long GetPitchMeasurement();
    unsigned long GetVolumeMeasurement();
    unsigned long GetQMeasurement();

    #ifdef ENABLE_AUDIO_FEEDBACK
    const float HZ_ADDVAL_FACTOR = 2.09785;
    const float MIDDLE_C = 261.6;
    void playNote(float hz, uint16_t milliseconds, uint8_t volume);
    #endif
  
    void delay_NOP(unsigned long time);

    #if CV_OUTPUT_MODE == CV_OUTPUT_MODE_LOG
    uint16_t log2U16 (uint16_t lin_input);
    #endif
};

#endif // _APPLICATION_H
