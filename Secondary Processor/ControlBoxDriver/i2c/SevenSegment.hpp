/*
 * SevenSegment.cpp
 *
 * Created: 4/25/2019 9:46:07 PM
 *  Author: John
 */
#ifndef SEVENSEGMENT_H
#define SEVENSEGMENT_H

#include "i2c.hpp"

#define LED_ON 1
#define LED_OFF 0

#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3



#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3

#define HT16K33_CMD_BRIGHTNESS 0xE0

#define SEVENSEG_DIGITS 5


// this is the raw HT16K33 controller
class Adafruit_LEDBackpack {
public:
	Adafruit_LEDBackpack(void);
	void begin(uint8_t _addr);
	void setBrightness(uint8_t b);
	void blinkRate(uint8_t b);
	void writeDisplay(void);
	void clear(void);

	uint16_t displaybuffer[8];

	void init(uint8_t a);
protected:
	uint8_t i2c_addr;
	I2C i2c_node;
	uint8_t message_data [32];
};


#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

class Adafruit_7segment : public Adafruit_LEDBackpack {
public:
	Adafruit_7segment(uint8_t address);
	unsigned int write(uint8_t c);

	void print(char, int = BYTE);
	void print(unsigned char, int = BYTE);
	void print(int, int = DEC);
	void print(unsigned int, int = DEC);
	void print(long, int = DEC);
	void print(unsigned long, int = DEC);
	void print(double, int = 2);
	void println(char, int = BYTE);
	void println(unsigned char, int = BYTE);
	void println(int, int = DEC);
	void println(unsigned int, int = DEC);
	void println(long, int = DEC);
	void println(unsigned long, int = DEC);
	void println(double, int = 2);
	void println(void);

	void writeDigitRaw(uint8_t x, uint8_t bitmask);
	void writeDigitNum(uint8_t x, uint8_t num, bool dot = false);
	void drawColon(bool state);
	void printNumber(long, uint8_t = 2);
	void printFloat(double, uint8_t = 2, uint8_t = DEC);
	void printError(void);

	void writeColon(void);

private:
	uint8_t position;
};
#endif