/*
 * config.h
 *
 * Put all your project settings here (defines, numbers, etc.)
 * configurations which are requiring knowledge of types etc.
 * shall be placed in z_config.ino (will be included at the end)
 *
 *  Created on: 12.05.2021
 *      Author: Marcel Licence
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/* enable the following to get a vt100 compatible output which can be displayed for example with teraterm pro */
//#define VT100_ENABLED

/* following can be activated to output the key function in the status output */
#define STATUS_SHOW_BUTTON_TEXT

/* use following when you are using the esp32 audio kit v2.2 */
#define ESP32_AUDIO_KIT /* project has not been tested on other hardware, modify on own risk */

/* this will force using const velocity for all notes, remove this to get dynamic velocity */
//#define MIDI_USE_CONST_VELOCITY

/* you can receive MIDI messages via serial-USB connection */
/*
 * you could use for example https://projectgus.github.io/hairless-midiserial/
 * to connect your MIDI device via computer to the serial port
 */
#define MIDI_RECV_FROM_SERIAL

/* activate MIDI via USB */
//#define MIDI_VIA_USB_ENABLED

#ifdef ESP32_AUDIO_KIT

/* on board led */
#define LED_PIN     22 // IO19 -> D5, IO22 -> D4

// set to pin connected to data input of WS8212 (NeoPixel) strip
#define LED_STRIP_PIN         12

/*
 * use the following when you've modified the audio kit like shown in video: https://youtu.be/r0af0DB1R68
 * move R66-70 to R60-R64 (0 Ohm resistors, you can also put a solder bridge)
 * place at R55-R59 a 1.8k resistor
 */
//#define AUDIO_KIT_BUTTON_ANALOG


#else /* ESP32_AUDIO_KIT */

/* on board led */
#define LED_PIN     2

/*
 * Define and connect your PINS to DAC here
 */

#ifdef I2S_NODAC
#define I2S_NODAC_OUT_PIN   22  /* noisy sound without DAC, add capacitor in series! */
#else
/*
 * pins to connect a real DAC like PCM5201
 */
#define I2S_BCLK_PIN    25
#define I2S_WCLK_PIN    27
#define I2S_DOUT_PIN    26
#endif



#endif /* ESP32_AUDIO_KIT */

/*
 * DIN MIDI Pinout
 */
#ifdef ESP32_AUDIO_KIT
#define MIDI_RX_PIN 18
//#define MIDI_SERIAL2_BAUDRATE   115200
#else
#define MIDI_RX_PIN 16
//#define TXD2 17
#endif


/*
 * You can modify the sample rate as you want
 */
#ifdef ESP32_AUDIO_KIT
#define SAMPLE_RATE 44100
#define SAMPLE_SIZE_16BIT
#else
#define SAMPLE_RATE 48000
#define SAMPLE_SIZE_32BIT
#endif



#endif /* CONFIG_H_ */