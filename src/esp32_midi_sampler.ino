/*
 * Copyright (c) 2021 Marcel Licence
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * veröffentlichten Version, weiter verteilen und/oder modifizieren.
 *
 * Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
 * OHNE JEDE GEWÄHR,; sogar ohne die implizite
 * Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Einzelheiten.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
 */

/*
 * this file should be opened with arduino, this is the main project file
 *
 * shown in: https://youtu.be/7uSobNW7_A4
 *
 * Some features listet here:
 * - loading/saving from/to sd
 * - add patch parameters saving
 * - buffered audio processing -> higher polyphony
 * - sd or littleFs usage (selectable)
 * - global pitchbend and modulation
 * - vu meter (using neopixel lib)
 * - adsr vca control
 * - file system selection -> littleFs support
 * - master reverb effect
 * - precise pitching of samples (just using simple float didn't work well)
 * - sound removal support -> defrag
 * - normalize record
 * - patch selection
 * - save with automatic increment of number in filename
 * - display / vt100 terminal support
 * - support of ST7735 160x80 compatible display -> update of Adafruit-ST7735-Library required
 * - allows using the AS5600 for scratching
 *
 * Author: Marcel Licence
 */

/*
 *
 /$$$$$$$$                     /$$       /$$                 /$$$$$$$   /$$$$$$  /$$$$$$$   /$$$$$$  /$$      /$$ /$$ /$$
| $$_____/                    | $$      | $$                | $$__  $$ /$$__  $$| $$__  $$ /$$__  $$| $$$    /$$$| $$| $$
| $$       /$$$$$$$   /$$$$$$ | $$$$$$$ | $$  /$$$$$$       | $$  \ $$| $$  \__/| $$  \ $$| $$  \ $$| $$$$  /$$$$| $$| $$
| $$$$$   | $$__  $$ |____  $$| $$__  $$| $$ /$$__  $$      | $$$$$$$/|  $$$$$$ | $$$$$$$/| $$$$$$$$| $$ $$/$$ $$| $$| $$
| $$__/   | $$  \ $$  /$$$$$$$| $$  \ $$| $$| $$$$$$$$      | $$____/  \____  $$| $$__  $$| $$__  $$| $$  $$$| $$|__/|__/
| $$      | $$  | $$ /$$__  $$| $$  | $$| $$| $$_____/      | $$       /$$  \ $$| $$  \ $$| $$  | $$| $$\  $ | $$
| $$$$$$$$| $$  | $$|  $$$$$$$| $$$$$$$/| $$|  $$$$$$$      | $$      |  $$$$$$/| $$  | $$| $$  | $$| $$ \/  | $$ /$$ /$$
|________/|__/  |__/ \_______/|_______/ |__/ \_______/      |__/       \______/ |__/  |__/|__/  |__/|__/     |__/|__/|__/


 */

/*
 * todos:
 * - better sound morphing - in progress
 * - fix of auto gain - improved
 * - add tremolo
 * - modulation after time
 * - harmonics?
 * - ignore other tags in wav files
 *
 * done:
 * - add wav saving to sd
 * - add wav loading from sd
 * - add patch parameter saving
 * - buffered audio processing poly from 6 to 14
 * - sd or littleFs usage
 * - pitchbend and modulation
 * - vu meter neopixel
 * - adsr control
 * - file system selection -> littleFs support
 * - add reverb effect
 * - increase of precision required of sample pitch
 * - sound removal support -> defrag
 * - normalize record
 * - patch selection
 * - save with automatic increment
 * - display
 * - simple scratching
 */

#include "config.h"

#include <Arduino.h>
#include <FS.h>
#include <LITTLEFS.h>
#include <SD_MMC.h>
#include <WiFi.h>

#ifdef AS5600_ENABLED
#include <Wire.h>
#endif

#include <Adafruit_ADS1X15.h>
#include <Adafruit_MPR121.h>

#include "web.h"

#define PIN_RECORD_BTN                   (22)

Adafruit_ADS1115 ads;
Adafruit_MPR121 cap = Adafruit_MPR121();
Adafruit_MPR121 cap2 = Adafruit_MPR121();

const float VPS = 4.096 / 32768.0; // volts per step
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
uint16_t lasttouched2 = 0;
uint16_t currtouched2 = 0;

uint8_t lastCh = 1;

const int na = 0;
const int8_t lastRecToReplace = 5;

/*
 * use this to activate auto loading
 */
//#define AUTO_LOAD_PATCHES_FROM_LITTLEFS
#define AUTO_LOAD_PATCHES_FROM_SD_MMC

/*
 * you can activate this but be careful
 * this will pass trough the mic signal to output during record
 * this may cause heavy feedbacks!
 */
//#define SAMPLER_PASS_TROUGH_DURING_RECORD

/*
 * Chip is ESP32D0WDQ5 (revision 1)
 * Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None

 * ref.: https://www.makerfabs.com/desfile/files/ESP32-A1S%20Product%20Specification.pdf

 * Board: ESP32 Dev Module
 * Flash Size: 32Mbit -> 4MB
 * RAM internal: 520KB SRAM
 * PSRAM: 4M (set to enabled!!!)
 *
 */

/* this is used to add a task to core 0 */
TaskHandle_t  Core0TaskHnd;

/* to avoid the high click when turning on the microphone */
static float click_supp_gain = 0.0f;

float *vuInL;
float *vuInR;
float *vuOutL;
float *vuOutR;

float *VuMeter_GetPtr(uint8_t index);

/* this application starts here */
void setup()
{
    // put your setup code here, to run once:
#ifdef BLINK_LED_PIN
    Blink_Setup();
#endif

    delay(500);

    Serial.begin(115200);
    Serial.println();
    Serial.printf("Total heap: %d\r\n", ESP.getHeapSize());
    Serial.printf("Free heap: %d\r\n", ESP.getFreeHeap());
    Serial.printf("Total PSRAM: %d\r\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d\r\n\n", ESP.getFreePsram());
    Serial.printf("Firmware started successfully\n");
    Serial.println();

    Wire.setClock(200000);
    ScanI2C();

    if (!ads.begin(ADS1X15_ADDRESS+1)) {
        Serial.println("Failed to initialize ADS.");
        while (1);
    }
    if (!cap.begin(MPR121_I2CADDR_DEFAULT)) {
        Serial.println("MPR121 not found, check wiring?");
        while (1);
    }
    if (!cap2.begin(MPR121_I2CADDR_DEFAULT+1)) {
        Serial.println("MPR121 not found, check wiring?");
        while (1);
    }

    // Config ADS to read exact voltage.
    ads.setGain(GAIN_ONE);

#ifdef AS5600_ENABLED
    //  digitalWrite(TFT_CS, HIGH);
    //  pinMode(TFT_CS, OUTPUT);

    Wire.setClock(I2C_SPEED);
#endif

    click_supp_gain = 0.0f;

    Status_Setup();

#ifdef ESP32_AUDIO_KIT
#ifdef ES8388_ENABLED
    ES8388_Setup();
#else
    ac101_setup();
    /* using mic as default source */
    ac101_setSourceMic();
#endif
#endif

#ifdef AS5600_ENABLED
    Wire.begin(I2C_SDA, I2C_SCL);
    AS5600_Setup();
#endif

    setup_i2s();
#ifdef ESP32_AUDIO_KIT
    button_setup();
#endif
    Sine_Init();

    Reverb_Setup();

    /*
     * setup midi module / rx port
     */
    Midi_Setup();

//#if 0
//    setup_wifi();
//#else
//    WiFi.mode(WIFI_OFF);
//#endif

#ifndef ESP8266
    btStop();
    // esp_wifi_deinit();
#endif

    Delay_Init();
    Delay_Reset();

    Sampler_Init();

    Serial.printf("ESP.getFreeHeap() %d\r\n", ESP.getFreeHeap());
    Serial.printf("ESP.getMinFreeHeap() %d\r\n", ESP.getMinFreeHeap());
    Serial.printf("ESP.getHeapSize() %d\r\n", ESP.getHeapSize());
    Serial.printf("ESP.getMaxAllocHeap() %d\r\n", ESP.getMaxAllocHeap());

    /* PSRAM will be fully used by the looper */
    Serial.printf("Total PSRAM: %d\r\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d\r\n", ESP.getFreePsram());

    vuInL = VuMeter_GetPtr(0);
    vuInR = VuMeter_GetPtr(1);
    vuOutL = VuMeter_GetPtr(6);
    vuOutR = VuMeter_GetPtr(7);

#ifdef AUTO_LOAD_PATCHES_FROM_LITTLEFS
    /* finally we can preload some data if available */
    PatchManager_SetDestination(0, 1);
    Sampler_LoadPatchFile("/samples/pet_bottle.wav");
#endif

    /* select sd card */
#ifdef AUTO_LOAD_PATCHES_FROM_SD_MMC
    PatchManager_SetDestination(1, 1);
    Sampler_LoadPatchFile("/samples/sinelong2.wav");
    Sampler_LoadPatchFile("/samples/sineshort2.wav");
    Sampler_LoadPatchFile("/samples/sleighbells.wav");
    Sampler_LoadPatchFile("/samples/woodblock.wav");
    Sampler_LoadPatchFile("/samples/cuckoo1.wav");
#endif

    /* we need a second task for the terminal output */
    xTaskCreatePinnedToCore(Core0Task, "Core0Task", 8000, NULL, 999, &Core0TaskHnd, 0);

    /* use this to easily test the output */
#if 0
    Sampler_NoteOn(0, 64, 1);
#endif

#ifdef AS5600_ENABLED
    /* starts first loaded sample and activates this for scratching */
    Sampler_SetScratchSample(0, 1);
#endif

    pinMode(PIN_RECORD_BTN, INPUT_PULLUP);

    Sampler_LoopAll(1, 1);
    Sampler_SetADSR_Attack(1, 0.0f);
    Sampler_SetADSR_Decay(1, 0.25f);
    Sampler_SetADSR_Sustain(1, 0.5f);
    Sampler_SetADSR_Release(1, 0.2f);
    Sampler_LoopStartC(1, 27/255.0f);
    Sampler_LoopStartF(1, 77/255.0f);
    Sampler_LoopEndC(1, 28/255.0f);
    Sampler_LoopEndF(1, 0/255.0f);
    Sampler_SetLoopEndMultiplier(1, 0/255.0f);

    Sampler_LoopRemove(3, 1);
    Sampler_SetADSR_Attack(3, 0.0f);
    Sampler_SetADSR_Decay(3, 1.00f);
    Sampler_SetADSR_Sustain(3, 1.0f);
    Sampler_SetADSR_Release(3, 1.0f);

    Sampler_LoopRemove(4, 1);
    Sampler_SetADSR_Attack(4, 0.0f);
    Sampler_SetADSR_Decay(4, 1.00f);
    Sampler_SetADSR_Sustain(4, 1.0f);
    Sampler_SetADSR_Release(4, 1.0f);

    Sampler_LoopAll(5, 1);
    Sampler_SetADSR_Attack(5, 0.0f);
    Sampler_SetADSR_Decay(5, 128/255.0f);
    Sampler_SetADSR_Sustain(5, 158/255.0f);
    Sampler_SetADSR_Release(5, 255/255.0f);
    Sampler_LoopStartC(5, 26/255.0f);
    Sampler_LoopStartF(5, 116/255.0f);
    Sampler_LoopEndC(5, 255/255.0f);
    Sampler_LoopEndF(5, 255/255.0f);
    Sampler_SetLoopEndMultiplier(5, 13/255.0f);

    App_SetOutputLevel(0, 2.5f);
    Status_Clear();
}

int cap_threshold = 50;
void App_CapThresholdsCore0()
{
    static int last_threshold = cap_threshold;
    if (cap_threshold != last_threshold) {
        last_threshold = cap_threshold;

        Serial.printf("Cap thresholds: %d %d", cap_threshold+5, cap_threshold);
        cap.setThresholds(cap_threshold + 5, cap_threshold);
        cap2.setThresholds(cap_threshold + 5, cap_threshold);
    }
}

void App_CapThresholds(uint8_t not_used, float value)
{
    cap_threshold = value * 240;
}

void App_RecordWait(uint8_t quarter, float value){
    Sampler_SelectRec(lastRecToReplace);
    Sampler_RemoveActiveRecording(na, 1);
    Sampler_RecordWait(na, value);
}


inline
void Core0TaskSetup()
{
#ifdef DISPLAY_160x80_ENABLED
    Display_Setup();
#ifdef SCREEN_ENABLED
    Screen_Setup();
#endif
    App_SetBrightness(0, 0.25);
#endif

    VuMeter_Init();

    Web_Setup();
    Web_AddSlider("Volume", App_SetOutputLevel, 255 * 35);
    Web_AddSlider("Reverb Level", Reverb_SetLevel);
    Web_AddSlider("Touch Sensitivity", App_CapThresholds);
    Web_AddLine();
    Web_AddButton("RecordWait", App_RecordWait);
    Web_AddButton("LoopAll", Sampler_LoopAll);
    Web_AddButton("LoopRemove", Sampler_LoopRemove);
    Web_AddButton("Panic", Sampler_Panic);
    Web_AddLine();
    Web_AddRecSelector();
    Web_AddSlider("Pitch", Sampler_SetPitch);
    Web_AddLine();
    Web_AddSlider("Attack", Sampler_SetADSR_Attack);
    Web_AddSlider("Decay", Sampler_SetADSR_Decay);
    Web_AddSlider("Sustain", Sampler_SetADSR_Sustain);
    Web_AddSlider("Release", Sampler_SetADSR_Release);
    Web_AddLine();
    Web_AddSlider("Delay Input Level", Delay_SetInputLevel);
    Web_AddSlider("Delay Feedback", Delay_SetFeedback);
    Web_AddSlider("Delay Level", Delay_SetLevel);
    Web_AddSlider("Delay Length", Delay_SetLength);
    Web_AddLine();
    Web_AddSlider("LoopStartC", Sampler_LoopStartC);
    Web_AddSlider("LoopStartF", Sampler_LoopStartF);
    Web_AddSlider("LoopEndC", Sampler_LoopEndC);
    Web_AddSlider("LoopEndF", Sampler_LoopEndF);
    Web_AddSlider("LoopMultiplier", Sampler_SetLoopEndMultiplier);
    Web_AddLine();
    Web_AddSlider("ModulationSpeed", Sampler_ModulationSpeed);
    Web_AddSlider("ModulationPitch", Sampler_ModulationPitch);
}


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    x = min(max(x, in_min), in_max);
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool isChanged(float first, float& second)
{
    if (abs(first - second) > 0.025) {
        second = first;
        return true;
    }
    return false;
}

void Update_Pots()
{
    static float lastPots[4];
    const float maxV = 3.3f;
    
    for (int p = 0; p < 4; ++p) {
        if (p == 2) continue;
        auto pot1 = ads.readADC_SingleEnded(p) * VPS;

        switch (p) {
            case 0:
                if (isChanged(pot1, lastPots[p])) {
                    Reverb_SetLevel(0, mapfloat(pot1, 1.3, maxV, 1.0f, 0.0f));
                }
                break;
            case 1:
                if (isChanged(pot1, lastPots[p])) {
                    Midi_PitchBend(0, mapfloat(pot1, 1.0, maxV, 0, 16384));
                }
                break;
            case 2:
                //if (isChanged(pot1, lastPots[p])) {
                //    auto val = mapfloat(pot1, 0, maxV, 0, 1.0f); 
                //}
                break;
            case 3:
                if (isChanged(pot1, lastPots[p])) {
                    auto val = mapfloat(pot1, 0.0, maxV, 0.01f, 20.0f); 
                    App_SetOutputLevel(1, val);
                    Status_ValueChangedFloat("volume 1", val);
                }
                break;
        }
    }

    for (int p = 0; p < 4; ++p) {
        Serial.printf("%.3f, ", lastPots[p]);
    }
    Serial.println();
}

struct CapMap {
    int8_t pin;
    int8_t channel;
    int8_t note;
};

// Rechts
struct CapMap mapping1[] = {
    { 0, 4, 69-12},    // A4, block
    { 1, 3, 72-12},    // C5, bells
    { 2, 1, 65-12},    // F4
    { 3, 1, 64-12},    // E4
    { 4, 1, 62-12},    // D4
    { 5, 1, 72-12},    // C5
    { 6, 1, 67-12},    // G4
    { 7, 4, 74-12},    // D5, block
    { 8, 1, 60-12},    // C4
    { 9, 3, 69-12},    // A4, bells
    { 10, 1, 69-12},   // A4
    { 11, 1, 71-12},   // B4
};

// Links
struct CapMap mapping2[] = {
    { 0, lastRecToReplace, 59},     // B3
    { 1, lastRecToReplace, 71},     // B4
    { 2, lastRecToReplace, 83},     // B5
    { 3, lastRecToReplace, 57},     // A3
    { 4, lastRecToReplace, 69},     // A4
    { 5, lastRecToReplace, 81},     // A5
    { 6, lastRecToReplace, 50},     // D3
    { 7, lastRecToReplace, 62},     // D4
    { 8, lastRecToReplace, 74},     // D5
    { 9, lastRecToReplace, 55-12},  // G2
    { 10, lastRecToReplace, 67-12}, // G3
    { 11, lastRecToReplace, 79-12}, // G4
};

void Read_Touches()
{
    App_CapThresholdsCore0();

    // Get the currently touched pads
    currtouched = cap.touched();
    currtouched2 = cap2.touched();
    
    for (uint8_t i=0; i<12; i++) {
        struct CapMap &map1 = mapping1[i];
        struct CapMap &map2 = mapping2[i];
        if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
            Sampler_NoteOn(map1.channel, map1.note, 1.0f);
            Serial.printf("  Note group 1, pin %d, channel %d, note %d\r\n", map1.pin, map1.channel, map1.note);
        }
        if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
            Sampler_NoteOff(map1.channel, map1.note);
        }
        if ((currtouched2 & _BV(i)) && !(lasttouched2 & _BV(i)) ) {
            Sampler_NoteOn(map2.channel, map2.note, 1.0f);
            Serial.printf("  Note group 2, pin %d, channel %d, note %d\r\n", map2.pin, map2.channel, map2.note);
        }
        if (!(currtouched2 & _BV(i)) && (lasttouched2 & _BV(i)) ) {
            Sampler_NoteOff(map2.channel, map2.note);
        }
    }

    lasttouched = currtouched;
    lasttouched2 = currtouched2;
}

void CheckRecordButton() {
    static bool lastState = HIGH;
    bool state = digitalRead(PIN_RECORD_BTN);

    if (state != lastState) {
        if (state == LOW) {
            Serial.printf("Pin 2 low");
            App_RecordWait(0, 1);
        }
        lastState = state;
    }
}


inline
void Core0TaskLoop()
{
    Status_Process();
#ifndef AS5600_ENABLED /* does not work together */
    VuMeter_Display();
#endif
#ifdef SCREEN_ENABLED
    Screen_Loop();
#endif
#ifdef DISPLAY_160x80_ENABLED
    Display_Draw();
#endif

    CheckRecordButton();
    Update_Pots();
    Read_Touches();
    Web_ProcessLoop();
}

void Core0Task(void *parameter)
{
    Core0TaskSetup();

    while (true)
    {
        Core0TaskLoop();

        /* this seems necessary to trigger the watchdog */
        delay(1);
        yield();
    }
}

float master_output_gain = 1.0f;

/* little enum to make switching more clear */
enum acSource
{
    acSrcLine,
    acSrcMic
};

/* line in is used by default, so it should not be changed here */
enum acSource selSource = acSrcLine;

/* be carefull when calling this function, microphones can cause very bad feedback!!! */
void App_ToggleSource(uint8_t channel, float value)
{
    if (value > 0)
    {
        switch (selSource)
        {
        case acSrcLine:
            click_supp_gain = 0.0f;
#ifdef AC101_ENABLED
            ac101_setSourceMic();
#endif
            selSource = acSrcMic;
            Status_TestMsg("Input: Microphone");
            break;
        case acSrcMic:
            click_supp_gain = 0.0f;
#ifdef AC101_ENABLED
            ac101_setSourceLine();
#endif
            selSource = acSrcLine;
            Status_TestMsg("Input: LineIn");
            break;
        }
    }
}

void App_SetOutputLevel(uint8_t not_used, float value)
{
    master_output_gain = value;
}

/*
 * this should avoid having a constant offset on our signal
 * I am not sure if that is required, but in case it can avoid early clipping
 */
static float fl_offset = 0.0f;
static float fr_offset = 0.0f;


static float fl_sample[SAMPLE_BUFFER_SIZE];
static float fr_sample[SAMPLE_BUFFER_SIZE];

#ifndef absf
#define absf(a) ((a>=0.0f)?(a):(-a))
#endif

/*
 * the main audio task
 */
inline void audio_task()
{
    memset(fl_sample, 0, sizeof(fl_sample));
    memset(fr_sample, 0, sizeof(fr_sample));

    i2s_read_stereo_samples_buff(fl_sample, fr_sample, SAMPLE_BUFFER_SIZE);

    for (int n = 0; n < SAMPLE_BUFFER_SIZE; n++)
    {
        /*
         * this avoids the high peak coming over the mic input when switching to it
         */
        fl_sample[n] *= click_supp_gain;
        fr_sample[n] *= click_supp_gain;

        if (click_supp_gain < 1.0f)
        {
            click_supp_gain += 0.00001f;
        }
        else
        {
            click_supp_gain = 1.0f;
        }

        /* make it a bit quieter */
        fl_sample[n] *= 0.5f;
        fr_sample[n] *= 0.5f;

        /*
         * this removes dc from signal
         */
        fl_offset = fl_offset * 0.99 + fl_sample[n] * 0.01;
        fr_offset = fr_offset * 0.99 + fr_sample[n] * 0.01;

        fl_sample[n] -= fl_offset;
        fr_sample[n] -= fr_offset;

        /*
         * put vu values to vu meters
         */
        *vuInL = max(*vuInL, absf(fl_sample[n]));
        *vuInR = max(*vuInR, absf(fr_sample[n]));
    }

    /*
     * main loop core
     */
    Sampler_Process(fl_sample, fr_sample, SAMPLE_BUFFER_SIZE);

    /*
     * little simple delay effect
     */
    Delay_Process_Buff(fl_sample, fr_sample, SAMPLE_BUFFER_SIZE);

    /*
     * add also some reverb
     */
    Reverb_Process(fl_sample, fr_sample, SAMPLE_BUFFER_SIZE);

    /*
     * apply master output gain
     */
    for (int n = 0; n < SAMPLE_BUFFER_SIZE; n++)
    {
        /* apply master_output_gain */
        fl_sample[n] *= master_output_gain;
        fr_sample[n] *= master_output_gain;

        /* output signal to vu meter */
        *vuOutL = max(*vuOutL, absf(fl_sample[n]));
        *vuOutR = max(*vuOutR, absf(fr_sample[n]));
    }

    /* function blocks and returns when sample is put into buffer */
    if (i2s_write_stereo_samples_buff(fl_sample, fr_sample, SAMPLE_BUFFER_SIZE))
    {
        /* nothing for here */
    }

    Status_Process_Sample(SAMPLE_BUFFER_SIZE);
}

/*
 * this function will be called once a second
 * call can be delayed when one operation needs more time (> 1/44100s)
 */
void loop_1Hz(void)
{
#if 0 /* use this line to to show analog button values to setup their values */
    printf("ADC: %d\n", analogRead(36));
#endif
}

void loop_100Hz(void)
{
    static uint32_t loop_cnt;

    loop_cnt += 1;
    if ((loop_cnt) >= 100)
    {
        loop_cnt = 0;
        loop_1Hz();
    }

    /*
     * Put your functions here which should be called 100 times a second
     */
#ifdef BLINK_LED_PIN
    Blink_Process(Sampler_IsRecording() ? 150 : 900);
#endif
#ifdef ESP32_AUDIO_KIT
    button_loop();
#endif
}

#ifdef AS5600_ENABLED
inline
void loop_4th()
{
#ifdef AS5600_ENABLED
#ifdef DISPLAY_160x80_ENABLED
    if (Display_Busy() == false)
#endif
    {
        AS5600_Loop();
        Sampler_SetPitchAbs(AS5600_GetPitch(4));
    }
#endif
}
#endif

/*
 * this is the main loop
 */
void loop()
{
    audio_task(); /* audio tasks blocks for one sample -> 1/44100s */

#ifdef AS5600_ENABLED
    static uint32_t prec4 = 0;
    prec4++;
    if ((prec4 % 4) == 0)
    {
        loop_4th();
    }
#endif

    VuMeter_Process(); /* calculates slow falling bars */

    static uint32_t loop_cnt;

    loop_cnt += SAMPLE_BUFFER_SIZE;
    if ((loop_cnt) >= (SAMPLE_RATE / 100))
    {
        loop_cnt = 0;
        loop_100Hz();
    }

    /*
     * doing midi only 64 times per sample cycle
     */
    Midi_Process();
}

uint8_t baseKey = 64 + 12 + 7; // c

#ifdef KEYS_TO_MIDI_NOTES

void App_Key1Down()
{
    Sampler_NoteOn(1, baseKey + 0, 1);
}

void App_Key2Down()
{
    Sampler_NoteOn(1, baseKey + 2, 1);
}

void App_Key3Down()
{
    Sampler_NoteOn(1, baseKey + 4, 1);
}

void App_Key4Down()
{
    Sampler_NoteOn(1, baseKey + 5, 1);
}

void App_Key5Down()
{
    Sampler_NoteOn(1, baseKey + 7, 1);
}

void App_Key6Down()
{
    Sampler_NoteOn(1, baseKey + 9, 1);
}

void App_Key1Up()
{

    Sampler_NoteOff(1, baseKey + 0);
}

void App_Key2Up()
{
    Sampler_NoteOff(1, baseKey + 2);
}

void App_Key3Up()
{
    Sampler_NoteOff(1, baseKey + 4);
}

void App_Key4Up()
{
    Sampler_NoteOff(1, baseKey + 5);
}

void App_Key5Up()
{
    Sampler_NoteOff(1, baseKey + 7);
}

void App_Key6Up()
{
    Sampler_NoteOff(1, baseKey + 9);
}
#else

enum keyMode_e
{
    keyMode_playbackChrom,
    keyMode_record,
    keyMode_playbackSample,
    keyMode_storage
};

enum keyMode_e currentKeyMode = keyMode_playbackChrom;

#ifdef AS5600_ENABLED

uint8_t scratchNote = 0;

void App_ButtonCbPlaySample(uint8_t key, uint8_t down)
{
    if (down > 0)
    {
        switch (key)
        {
        case 0:
            scratchNote++;
            Sampler_SetScratchSample(scratchNote, 1);
            break;

        case 1:
            if (scratchNote > 0)
            {
                scratchNote--;
            }
            Sampler_SetScratchSample(scratchNote, 1);
            break;
        }
    }
    switch (key)
    {
    case 2:
        delay(250); /* to avoid recording the noise of the button */
        Sampler_RecordWait(0, down);
        break;

    case 3:
        //Sampler_Record(0, down);
        if (down > 0)
        {
            AS5600_SetPitchOffset(1.0f);
        }
        else
        {
            AS5600_SetPitchOffset(-100.0f);
        }
        break;
    }
}

#else


void App_ButtonCb(uint8_t key, uint8_t down)
{
    if ((key == 0) && (down > 0))
    {
        switch (currentKeyMode)
        {
        case keyMode_storage:
            currentKeyMode = keyMode_record;
#ifdef DISPLAY_160x80_ENABLED
            Display_SetHeader("Record");
#endif
            Status_LogMessage("Record");

            Status_SetKeyText(0, "1:select");
            Status_SetKeyText(1, "2:rWait");
            Status_SetKeyText(2, "3:record");
            Status_SetKeyText(3, "4:littFs");
            Status_SetKeyText(4, "5:sdMmc");
            Status_SetKeyText(5, "6:save");

            break;
        case keyMode_record:
            currentKeyMode = keyMode_playbackSample;
#ifdef DISPLAY_160x80_ENABLED
            Display_SetHeader("Sample playback");
#endif
            Status_LogMessage("Sample playback");

            Status_SetKeyText(0, "1:select");
            Status_SetKeyText(1, "2:smpl1");
            Status_SetKeyText(2, "3:smpl2");
            Status_SetKeyText(3, "4:smpl3");
            Status_SetKeyText(4, "5:smpl4");
            Status_SetKeyText(5, "6:smpl5");
            break;
        case keyMode_playbackSample:
            currentKeyMode = keyMode_playbackChrom;
#ifdef DISPLAY_160x80_ENABLED
            Display_SetHeader("Chromatic playback");
#endif
            Status_LogMessage("Chromatic playback");

            Status_SetKeyText(0, "1:select");
            Status_SetKeyText(1, "2:c");
            Status_SetKeyText(2, "3:c#");
            Status_SetKeyText(3, "4:d");
            Status_SetKeyText(4, "5:d#");
            Status_SetKeyText(5, "6:e");
            break;

        case keyMode_playbackChrom:
            currentKeyMode = keyMode_storage;
#ifdef DISPLAY_160x80_ENABLED
            Display_SetHeader("Sample loader");
#endif
            Status_LogMessage("Sample loader");

            Status_SetKeyText(0, "1:select");
            Status_SetKeyText(1, "2:next");
            Status_SetKeyText(2, "3:prev");
            Status_SetKeyText(3, "4:littFs");
            Status_SetKeyText(4, "5:sdMmc");
            Status_SetKeyText(5, "6:load");
            break;
        }
    }
    else
    {
        switch (currentKeyMode)
        {
        case keyMode_playbackChrom:
            if (down > 0)
            {
                Sampler_NoteOn(lastCh, baseKey + key, 1);
            }
            else
            {
                Sampler_NoteOff(lastCh, baseKey + key);
            }
            break;
        case keyMode_record:
            switch (key)
            {
            case 1:
                delay(250); /* to avoid recording the noise of the button */
                Sampler_SelectRec(lastRecToReplace);
                Sampler_RemoveActiveRecording(na, 1);
                Sampler_RecordWait(na, down);
                break;
            case 2:
                Sampler_SelectRec(lastRecToReplace);
                Sampler_RemoveActiveRecording(na, 1);
                Sampler_Record(na, down);
                break;
            case 3:
                PatchManager_SetDestination(0, 0);
                break;
            case 4:
                PatchManager_SetDestination(0, 1);
                break;
            case 5:
                Sampler_SavePatch(0, down);
                break;

            }
            break;
        case keyMode_playbackSample:
            if (down > 0)
            {
                Sampler_NoteOn(0, key - 1, 1);
                lastCh = key;
            }
            else
            {
                Sampler_NoteOff(0, key - 1);
            }
            break;
        case keyMode_storage:
            if (down > 0)
            {
                switch (key)
                {
                case 1:
                    PatchManager_FileIdxInc(0, 1);
                    break;
                case 2:
                    PatchManager_FileIdxDec(0, 1);
                    break;
                case 3:
                    PatchManager_SetDestination(0, 0);
                    break;
                case 4:
                    PatchManager_SetDestination(0, 1);
                    break;
                case 5:
                    Sampler_SelectRec(lastRecToReplace);
                    Sampler_RemoveActiveRecording(na, 1);
                    Sampler_LoadPatch(na, 1);
                    break;
                }
            }
            break;
        }
    }
}

#endif
#endif

void App_SetBrightness(uint8_t unused, float value)
{
#ifdef DISPLAY_160x80_ENABLED
    Display_SetBacklight(value * 255.0f);
#endif
    VuMeter_SetBrighness(unused, value / 8.0f);
}


/*
 * Test functions
 */

void  ScanI2C(void)
{

    Wire.begin(I2C_SDA, I2C_SCL);

    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
}
