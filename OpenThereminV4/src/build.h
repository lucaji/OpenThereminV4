/**
 * @file build.h
 * @brief Build options for OpenTheremin
 *
 * Define directives for custom build options
 *
 * REVISION HISTORY
 * July 2025 Luca Cipressi (lucaji.github.io) added extended serial modes and interpolation.
 * (c) GNU GPL v3 or later.
 */

#ifndef _BUILD_H_
#define _BUILD_H_

// SERIAL_MODE sets the USB Serial speed according selected functionality
// for better audio output quality, set it off as it saves precious resources.
// the MIDI mode DOES NOT enable the OpenTheremin as a MIDI compliant USB device
// you will need a software on your PC to convert USB serial MIDI data into a virtual MIDI port.
#define SERIAL_MODE_OFF 0               // disables the use of the serial port entirely
#define SERIAL_MODE_DEBUG 1             // initializes the serial port at 57600 bps and emits debug information
#define SERIAL_MODE_MIDI 2              // Initializes the serial port at 31250 bps and emits MIDI note (experimental)
// select one option from above
#define SERIAL_MODE SERIAL_MODE_OFF

#if SERIAL_MODE == SERIAL_MODE_DEBUG
  #define DEBUG_PRINT(x)     Serial.print(x)
  #define DEBUG_PRINTLN(x)   Serial.println(x)
#else
  #define DEBUG_PRINT(x)     ((void)0)
  #define DEBUG_PRINTLN(x)   ((void)0)
#endif

/*
 * AUDIO FEEDBACK
 * 
 * 
 */
// if defined, will emit sound feedback
// comment it out for silent operation (only LED feedback)
//#define ENABLE_AUDIO_FEEDBACK

/*
 * CV_OUTPUT_MODE sets options for the CV output jack
 *
 * for better audio output quality, set it off as it saves precious resources
 * especially if you don't need CV/GATE outputs. Refer to each option's comment
 * for description.
 * 
 * 
 */
#define CV_OUTPUT_MODE_OFF 0            // disables the CV output - this saves resources - for a slightly better audio quality if not needed at all.
#define CV_OUTPUT_MODE_LOG 1            // uses a logarithmic curve for CV output (1V/Oct for Moog & Roland)
#define CV_OUTPUT_MODE_LINEAR 2         // uses a linear transfer function for CV output (819Hz/V for Korg & Yamaha)
#define CV_OUTPUT_MODE CV_OUTPUT_MODE_OFF

/*
 * GATE OUTPUT
 *
 */
// Set the trigger levels for the Gate signal (0 to 127 in preparation for later MIDI extensions)
#define GATE_ON  20 // That's the level which will drive the Gate high when volume increases from lower
#define GATE_OFF 16 // That's the level which will drive the Gate low when volume decreases from higher
// Making both values equal risk the gate signal to bounce, leave at least 4 (hysteresis) between both.
// Set both to 128 to disable the Gate Signal.

/*
 * SAMPLE INTERPOLATION (EXPERIMENTAL)
 * 
 * these switches will activate different smoothing algorithms trading off the
 * computational efficiency and the resulting harmonic content of the output wavetable
 * given the limitations of the wavetable size and the strict timing needed for the
 * DDS generator to complete the ISR routine especially on the 328p at 16MHz
 * 
 * - INTERPOLATION_NONE
 *   the best results are guaranteed with the original algorithm used in OpenTheremin
 *   because the table step matches a power-of-2 accumulator (phase alignment),
 *   No risk of overshooting, clipping, or interpolator curve "ripples",
 *   Less CPU jitter: less timing variability = more phase-coherent output.
 *   -> Optimal harmonic distortion (clean FFT)
 *   -> High responsiveness (fast reaction time)
 *   -> Higher frequency limit in pitch generation (4kHz)
 * 
 * - INTERPOLATION_LINEAR
 *   smooths the lower frequencies, but introduces some harmonic content
 * 
 * - INTERPOLATION_HERMITE - INTERPOLATION_CATMULL_ROM
 *   very high computation budget and introduces significant latency
 *   At 4 kHz pitch = 128 samples/ms = 32 µs per sample budget
 *   Catmull/Hermite easily consume 20–28 µs, leaving very little headroom.
 *   Lower max pitch:
 *   Due to ISR latency, sample generation can't keep up → dropped samples = lower actual frequency.
 *   Both introduce curvature that adds non-sinusoidal partials.
 * 
 * - INTERPOLATION_MODE_INTERACTIVE
 *   for testing and experimental purposes, it will advance each of the previous modes
 *   everytime the user switches between MUTE and UNMUTE (by touching once the touch button)
 *   make sure you activate the serial port for debug to see which mode is engaged.
 * 
 * [Hermite Interpolation](https://en.wikipedia.org/wiki/Hermite_interpolation)
 * [Catmull-Rom Spline](https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull%E2%80%93Rom_spline)
 * 
 */
#define INTERPOLATION_NONE        0             // best choice for 328p, clean harmonic content, highest responsiveness, higher pitch range
#define INTERPOLATION_LINEAR      1             // smooths low-frequency pitches, introduces slight harmonic content
#define INTERPOLATION_HERMITE     2             // heavy computing budget - introduces latency
#define INTERPOLATION_CATMULL_ROM 3             // heavy computing budget - introduces latency
#define INTERPOLATION_MODE_INTERACTIVE 127      // for testing purposes, includes all the above code, but at each mute->unmute operation it will rotate between the modes.
#define INTERPOLATION_MODE INTERPOLATION_NONE

/*
 * INCLUDES A PRECISE PURE SINEWAVE IF DEFINED
 * as waveform at index 0 (the very first)
 * 
 * useful for test purposes (FFT) or to be played as a very neutral timbre
 * 
 * this sinewave generates an almost pure harmonic content
 * it still will have limitations because of the limited (1024 points) table
 * and the strict timing implemented in this current OpenTheremin firmware.
 * 
 * none of standard wavetables included with OpenTheremin
 * are not perfect sinewaves, as the original Theremin timbre
 * has its own timbre. the OT wavetables mimic those original
 * sounding waveforms pretty accurately.
 * 
 */
//#define WAVEFORM_INCLUDE_PURE_SINE

/*
 * PITCH and VOLUME measurement debounce strategy
 * 
 * - LEGACY debounces over 5 pulses before flagging the reading as ready.
 * - EDGE_DETECT is faster and does not block interrupts.
 */
#define DDS_DEBOUNCE_MODE_LEGACY 0
#define DDS_DEBOUNCE_MODE_EDGE_DETECT 1
#define DDS_DEBOUNCE_MODE DDS_DEBOUNCE_MODE_EDGE_DETECT

#endif // _BUILD_H_
