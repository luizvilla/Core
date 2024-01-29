#ifndef LCDLIB_HPP_
#define LCDLIB_HPP_

#include <stddef.h>
#include <stdarg.h>
#include <inttypes.h>

#include "SpinAPI.h"
#include "TaskAPI.h"
//
#define LCD_8_BITS_MODE 0
#define LCD_4_BITS_MODE 1

//Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00
class LCD
{
	public:
		LCD(pin_t rs, pin_t e, pin_t d0, pin_t d1, pin_t d2, pin_t d3,
		pin_t d4, pin_t d5, pin_t d6, pin_t d7);
		LCD(pin_t rs, pin_t e, pin_t d4, pin_t d5, pin_t d6, pin_t d7);
		~LCD();
		void begin(uint8_t cols, uint8_t lines);
		void clear();
		void home();
		void setCursor(uint8_t col, uint8_t row);
		void noDisplay();
		void display();
		void noCursor();
		void cursor();
		void noBlink();
		void blink();
		void scrollDisplayLeft();
		void scrollDisplayRight();
		void leftToRight();
		void rightToLeft();
		void autoscroll();
		void noAutoscroll();
		void createChar(uint8_t location, uint8_t charmap[]);
		size_t write(uint8_t value);
		void print(char *str);
		void printf(const char* format, ...);
	private:
		inline void command(uint8_t value);
		void send(uint8_t value, uint8_t mode);
		void pulseEnable();
		void write4bits(uint8_t value);
		void write8bits(uint8_t value);

		uint8_t _mode; // 4 or 8 bits mode
		pin_t _rs; // LOW: command. HIGH: character.
		pin_t _e; // activated by a HIGH pulse.
		pin_t _data_pins[8];


		uint8_t _displayfunction;
		uint8_t _displaycontrol;
		uint8_t _displaymode;

		uint8_t _numlines;
		uint8_t _row_offsets[4];

};
#endif /* !LCDLIB_HPP_*/