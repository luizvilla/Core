#include "LCDLib.hpp"

LCD::LCD(pin_t rs, pin_t e, pin_t d0, pin_t d1, pin_t d2, pin_t d3,
		pin_t d4, pin_t d5, pin_t d6, pin_t d7)
{
	_mode = LCD_8_BITS_MODE;
	_rs = rs;
	_e = e;
	_data_pins[0] = d0;
	_data_pins[1] = d1;
	_data_pins[2] = d2;
	_data_pins[3] = d3; 
	_data_pins[4] = d4;
	_data_pins[5] = d5;
	_data_pins[6] = d6;
	_data_pins[7] = d7; 
	
}

LCD::LCD(pin_t rs, pin_t e, pin_t d4, pin_t d5, pin_t d6, pin_t d7)
{
	_mode = LCD_4_BITS_MODE;
	_mode = LCD_8_BITS_MODE;
	_rs = rs;
	_e = e;
	_data_pins[0] = d4;
	_data_pins[1] = d5;
	_data_pins[2] = d6;
	_data_pins[3] = d7; 
}

LCD::~LCD()
{
}

void LCD::begin(uint8_t cols, uint8_t lines)
{
	if (lines > 1)
	{
		_displayfunction |= LCD_2LINE;
	}
	_numlines = lines;

	_row_offsets[0] = 0x00;
	_row_offsets[1] = 0x40;
	_row_offsets[2] = 0x00 + cols;
	_row_offsets[3] = 0x40 + cols;
	//   // for some 1 line displays you can select a 10 pixel high font
	//   if ((dotsize != LCD_5x8DOTS) && (lines == 1)) {
	//     _displayfunction |= LCD_5x10DOTS;
	//   }

	spin.gpio.configurePin(_rs, OUTPUT); // pinMode(_rs_pin, OUTPUT);
	
	// we can save 1 pin by not using RW. Indicate by passing 255 instead of pin#
	//   if (_rw != 255) { 
	// 	gpio.configurePin(_rw, OUTPUT); // pinMode(_rw_pin, OUTPUT);
	//   }
	spin.gpio.configurePin(_e, OUTPUT); // pinMode(_enable_pin, OUTPUT);


	// Do these once, instead of every time a character is drawn for speed reasons.
	for (int i=0; i<((_displayfunction & LCD_8BITMODE) ? 8 : 4); ++i)
	{
		spin.gpio.configurePin(_data_pins[i], OUTPUT); // pinMode(_data_pins[i], OUTPUT);
	} 

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40 ms after power rises above 2.7 V
	// before sending commands. Arduino can turn on way before 4.5 V so we'll wait 50
	k_sleep(K_USEC(50000)); // delayMicroseconds(50000); 
	// Now we pull both RS and R/W low to begin commands
	spin.gpio.resetPin(_rs); // digitalWrite(_rs_pin, LOW);
	spin.gpio.resetPin(_e); // digitalWrite(_enable_pin, LOW);

	// if (_rw != 255) { 
	// 	gpio.resetPin(_rw); // digitalWrite(_rw_pin, LOW);
	// }

	k_sleep(K_USEC(4500)); //delayMicroseconds(4500); // wait min 4.1ms
	this->write4bits(0x02); 

	//put the LCD into 4 bit or 8 bit mode
	if (! (_displayfunction & LCD_8BITMODE))
	{
		// this is according to the Hitachi HD44780 datasheet
		// figure 24, pg 46

		// we start in 8bit mode, try to set 4 bit mode
		this->write4bits(0x03);
		k_sleep(K_USEC(4500)); //delayMicroseconds(4500); // wait min 4.1ms

		// second try
		this->write4bits(0x03);
		k_sleep(K_USEC(4500)); //delayMicroseconds(4500); // wait min 4.1ms
		
		// third go!
		this->write4bits(0x03); 
		k_sleep(K_USEC(150)); //delayMicroseconds(150);

		// finally, set to 4-bit interface
		this->write4bits(0x02); 
	}
	else
	{
		// this is according to the Hitachi HD44780 datasheet
		// page 45 figure 23

		// Send function set command sequence
		this->command(LCD_FUNCTIONSET | _displayfunction);
		k_sleep(K_USEC(4500)); //delayMicroseconds(4500);  // wait more than 4.1 ms

		// second try
		this->command(LCD_FUNCTIONSET | _displayfunction);
		k_sleep(K_USEC(150)); //delayMicroseconds(150);
	

		// third go
		this->command(LCD_FUNCTIONSET | _displayfunction);
	}

  // finally, set # lines, font size, etc.
	this->command(LCD_FUNCTIONSET | _displayfunction);  

	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
	this->display();

	// clear it off
	this->clear();

	// Initialize to default text direction (for romance languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	// set the entry mode
	this->command(LCD_ENTRYMODESET | _displaymode);

}


void LCD::clear()
{
	this->command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
	k_sleep(K_USEC(2000)); //delayMicroseconds(2000);  // this command takes a long time!
}

void LCD::home()
{
	this->command(LCD_RETURNHOME);  // set cursor position to zero
	k_sleep(K_USEC(2000)); //delayMicroseconds(2000);  // this command takes a long time!
}

void LCD::setCursor(uint8_t col, uint8_t row)
{
	const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
	if ( row >= max_lines ) {
		row = max_lines - 1;    // we count rows starting w/ 0
	}
	if ( row >= _numlines ) {
		row = _numlines - 1;    // we count rows starting w/ 0
	}

	this->command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}

// Turn the display on/off (quickly)
void LCD::noDisplay()
{
	_displaycontrol &= ~LCD_DISPLAYON;
	this->command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD::display()
{
	_displaycontrol |= LCD_DISPLAYON;
	this->command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LCD::noCursor()
{
	_displaycontrol &= ~LCD_CURSORON;
	this->command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD::cursor()
{
	_displaycontrol |= LCD_CURSORON;
	this->command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void LCD::noBlink()
{
	_displaycontrol &= ~LCD_BLINKON;
	this->command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD::blink() {
	_displaycontrol |= LCD_BLINKON;
	this->command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void LCD::scrollDisplayLeft()
{
	this->command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void LCD::scrollDisplayRight()
{
	this->command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LCD::leftToRight()
{
	_displaymode |= LCD_ENTRYLEFT;
	this->command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void LCD::rightToLeft()
{
	this->_displaymode &= ~LCD_ENTRYLEFT;
	this->command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void LCD::autoscroll()
{
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	this->command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void LCD::noAutoscroll()
{
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	this->command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LCD::createChar(uint8_t location, uint8_t charmap[])
{
	location &= 0x7; // we only have 8 locations 0-7
	this->command(LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++)
	{
		this->write(charmap[i]);
	}
}

size_t LCD::write(uint8_t value)
{
	this->send(value, 1);
	return 1; // assume success
}

void LCD::print(char *str)
{
	for (uint16_t i = 0; i < str[i] != '\0'; i++)
	{
		this->write((uint8_t)str[i]);
	}
}


void LCD::printf(const char* format, ...)
{
	this->setCursor(0, 0);
    va_list argptr;
    va_start(argptr, format);
	while (*format != '\0')
	{


		if (*format == '%')
		{
			format++;
			uint8_t precision = -1;
			if (*format == '.')
			{
				precision = 0;
				format++;
				while (*format >= '0' && *format <= '9')
				{
					precision = precision * 10 + (*format - '0');
					format++;
				}
			}

			char txtBuff[16];
			char preBuff[8];

			switch (*format) {
				case 'd':
					snprintk(txtBuff, 16, "%d", va_arg(argptr, int));
					this->print(txtBuff);
					break;
				case 'f':
					if (precision != -1)
					{
						//had to spit in two. snprintk is not like snprintf
						snprintk(preBuff, 8, "%.");
						snprintk(preBuff, 8, "%s%df", preBuff, precision); 
						snprintk(txtBuff, 16, preBuff, va_arg(argptr, double));
					}
					else
					{
						snprintk(txtBuff, 16, "%f", va_arg(argptr, int));
					}

					this->print(txtBuff);
					break;
				case 's':
					this->print(va_arg(argptr, char *));
					break;
				default:
					this->write('%');
					this->write(*format);
			}
		}
		else if (*format == '\n')
		{
			this->setCursor(0, 1);
		}
		else
		{
			this->write(*format);
		}
		format++;
	}
	va_end(argptr);
}
////////////////////////// PRIVATE //////////////////////////

inline void LCD::command(uint8_t value)
{
	this->send(value, 0);
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void LCD::send(uint8_t value, uint8_t mode)
{
	spin.gpio.writePin(_rs, mode); //digitalWrite(_rs_pin, mode);
 	 // if there is a RW pin indicated, set it low to Write
	//   if (_rw_pin != 255) { 
	// 	gpio.resetPin(_rw);  //digitalWrite(_rw_pin, LOW);
	//   }

	if (_displayfunction & LCD_8BITMODE)
	{
		this->write8bits(value); 
	} else {
		this->write4bits(value>>4);
		this->write4bits(value);
	}
}

void LCD::pulseEnable()
{
	spin.gpio.resetPin(_e); //digitalWrite(_enable_pin, LOW);
	k_sleep(K_USEC(1)); //delayMicroseconds(1);
	spin.gpio.setPin(_e); //digitalWrite(_enable_pin, HIGH);
	k_sleep(K_USEC(1)); //delayMicroseconds(1); // enable pulse must be >450 ns
	spin.gpio.resetPin(_e); //digitalWrite(_enable_pin, LOW);
	k_sleep(K_USEC(100)); //delayMicroseconds(100);   // commands need >37 us to settle
}

void LCD::write4bits(uint8_t value)
{
	for (int i = 0; i < 4; i++)
	{
		spin.gpio.writePin(_data_pins[i], (value >> i) & 0x01); //digitalWrite(_data_pins[i], (value >> i) & 0x01);
	}
	this->pulseEnable();
}

void LCD::write8bits(uint8_t value)
{
	for (int i = 0; i < 8; i++)
	{
		spin.gpio.writePin(_data_pins[i], (value >> i) & 0x01); //digitalWrite(_data_pins[i], (value >> i) & 0x01);
	}
	this->pulseEnable();
}
