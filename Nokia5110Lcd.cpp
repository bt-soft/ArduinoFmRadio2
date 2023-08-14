/*
 * Nokia5110Lcd.cpp
 *
 *  Created on: 2022. okt. 29.
 *      Author: BT
 */
#include <avr/pgmspace.h>
#include "Nokia5110Lcd.h"

#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask
#define pulseClock cbi(P_SCK, B_SCK); asm ("nop"); sbi(P_SCK, B_SCK)
#define resetLCD sbi(P_DC, B_DC); sbi(P_MOSI, B_MOSI); sbi(P_SCK, B_SCK); sbi(P_CS, B_CS); cbi(P_RST, B_RST); delay(10); sbi(P_RST, B_RST)

#define fontbyte(x) pgm_read_byte(&cfont.font[x])
#define bitmapbyte(x) pgm_read_byte(&bitmap[x])
#define bitmapdatatype uint8_t*

#define LCD_COMMAND 0
#define LCD_DATA 1

// PCD8544 Commandset
// ------------------
// General commands
#define PCD8544_POWERDOWN			0x04
#define PCD8544_ENTRYMODE			0x02
#define PCD8544_EXTENDEDINSTRUCTION	0x01
#define PCD8544_DISPLAYBLANK		0x00
#define PCD8544_DISPLAYNORMAL		0x04
#define PCD8544_DISPLAYALLON		0x01
#define PCD8544_DISPLAYINVERTED		0x05
// Normal instruction set
#define PCD8544_FUNCTIONSET			0x20
#define PCD8544_DISPLAYCONTROL		0x08
#define PCD8544_SETYADDR			0x40
#define PCD8544_SETXADDR			0x80
// Extended instruction set
#define PCD8544_SETTEMP				0x04
#define PCD8544_SETBIAS				0x10
#define PCD8544_SETVOP				0x80
// Display presets
#define LCD_BIAS					0x03	// Range: 0-7 (0x00-0x07)
#define LCD_TEMP					0x02	// Range: 0-3 (0x00-0x03)
#define LCD_CONTRAST				0x46	// Range: 0-127 (0x00-0x7F)

/**
 *
 */
Nokia5110Lcd::Nokia5110Lcd(int SCK, int MOSI, int DC, int RST, int CS) {
	P_SCK = portOutputRegister(digitalPinToPort(SCK));
	B_SCK = digitalPinToBitMask(SCK);
	P_MOSI = portOutputRegister(digitalPinToPort(MOSI));
	B_MOSI = digitalPinToBitMask(MOSI);
	P_DC = portOutputRegister(digitalPinToPort(DC));
	B_DC = digitalPinToBitMask(DC);
	P_RST = portOutputRegister(digitalPinToPort(RST));
	B_RST = digitalPinToBitMask(RST);
	P_CS = portOutputRegister(digitalPinToPort(CS));
	B_CS = digitalPinToBitMask(CS);

	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);
	pinMode(DC, OUTPUT);
	pinMode(RST, OUTPUT);
	pinMode(CS, OUTPUT);

	SCK_Pin = SCK;
	RST_Pin = RST;
}

void Nokia5110Lcd::_LCD_Write(unsigned char data, unsigned char mode) {
	cbi(P_CS, B_CS);

	if (mode == LCD_COMMAND)
		cbi(P_DC, B_DC);
	else
		sbi(P_DC, B_DC);

	for (unsigned char c = 0; c < 8; c++) {
		if (data & 0x80)
			sbi(P_MOSI, B_MOSI);
		else
			cbi(P_MOSI, B_MOSI);
		data = data << 1;
		pulseClock
		;
	}

	sbi(P_CS, B_CS);
}

/**
 *
 */
void Nokia5110Lcd::init(int contrast = 70) {
	if (contrast > 0x7F)
		contrast = 0x7F;

	if (contrast < 0)
		contrast = 0;

	resetLCD
	;

	_LCD_Write(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION, LCD_COMMAND);
	_LCD_Write(PCD8544_SETVOP | contrast, LCD_COMMAND);
	_LCD_Write(PCD8544_SETTEMP | LCD_TEMP, LCD_COMMAND);
	_LCD_Write(PCD8544_SETBIAS | LCD_BIAS, LCD_COMMAND);
	_LCD_Write(PCD8544_FUNCTIONSET, LCD_COMMAND);
	_LCD_Write(PCD8544_SETYADDR, LCD_COMMAND);
	_LCD_Write(PCD8544_SETXADDR, LCD_COMMAND);
	_LCD_Write(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL, LCD_COMMAND);

	clrScr();
	update();
	cfont.font = 0;
	_sleep = false;
	_contrast = contrast;
}

/**
 *
 */
void Nokia5110Lcd::clrScr() {
	for (int c = 0; c < 504; c++)
		scrbuf[c] = 0;
}

/**
 *
 */
void Nokia5110Lcd::update() {
	if (_sleep == false) {
		_LCD_Write(PCD8544_SETYADDR, LCD_COMMAND);
		_LCD_Write(PCD8544_SETXADDR, LCD_COMMAND);
		for (int b = 0; b < 504; b++)
			_LCD_Write(scrbuf[b], LCD_DATA);
	}
}

/**
 *
 */
void Nokia5110Lcd::setContrast(int contrast) {
	if (contrast > 0x7F)
		contrast = 0x7F;
	if (contrast < 0)
		contrast = 0;
	_LCD_Write(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION, LCD_COMMAND);
	_LCD_Write(PCD8544_SETVOP | contrast, LCD_COMMAND);
	_LCD_Write(PCD8544_FUNCTIONSET, LCD_COMMAND);
	_contrast = contrast;
}

/**
 *
 */
void Nokia5110Lcd::enableSleep() {
	_sleep = true;
	_LCD_Write(PCD8544_SETYADDR, LCD_COMMAND);
	_LCD_Write(PCD8544_SETXADDR, LCD_COMMAND);
	for (int b = 0; b < 504; b++)
		_LCD_Write(0, LCD_DATA);
	_LCD_Write(PCD8544_FUNCTIONSET | PCD8544_POWERDOWN, LCD_COMMAND);
}

/**
 *
 */
void Nokia5110Lcd::disableSleep() {
	_sleep = false;
	_LCD_Write(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION, LCD_COMMAND);
	_LCD_Write(PCD8544_SETVOP | _contrast, LCD_COMMAND);
	_LCD_Write(PCD8544_SETTEMP | LCD_TEMP, LCD_COMMAND);
	_LCD_Write(PCD8544_SETBIAS | LCD_BIAS, LCD_COMMAND);
	_LCD_Write(PCD8544_FUNCTIONSET, LCD_COMMAND);
	_LCD_Write(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL, LCD_COMMAND);
	update();
}

/**
 *
 */
void Nokia5110Lcd::fillScr() {
	for (int c = 0; c < 504; c++)
		scrbuf[c] = 255;
}

void Nokia5110Lcd::invert(bool mode) {
	if (mode == true)
		_LCD_Write(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYINVERTED, LCD_COMMAND);
	else
		_LCD_Write(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL, LCD_COMMAND);
}

void Nokia5110Lcd::setPixel(uint16_t x, uint16_t y) {
	int by, bi;

	if ((x >= 0) and (x < 84) and (y >= 0) and (y < 48)) {
		by = ((y / 8) * 84) + x;
		bi = y % 8;

		scrbuf[by] = scrbuf[by] | (1 << bi);
	}
}

void Nokia5110Lcd::clrPixel(uint16_t x, uint16_t y) {
	int by, bi;

	if ((x >= 0) and (x < 84) and (y >= 0) and (y < 48)) {
		by = ((y / 8) * 84) + x;
		bi = y % 8;

		scrbuf[by] = scrbuf[by] & ~(1 << bi);
	}
}

void Nokia5110Lcd::invPixel(uint16_t x, uint16_t y) {
	int by, bi;

	if ((x >= 0) and (x < 84) and (y >= 0) and (y < 48)) {
		by = ((y / 8) * 84) + x;
		bi = y % 8;

		if ((scrbuf[by] & (1 << bi)) == 0)
			scrbuf[by] = scrbuf[by] | (1 << bi);
		else
			scrbuf[by] = scrbuf[by] & ~(1 << bi);
	}
}

void Nokia5110Lcd::invertText(bool mode) {
	if (mode == true)
		cfont.inverted = 1;
	else
		cfont.inverted = 0;
}

void Nokia5110Lcd::print(char *st, int x, int y) {
	unsigned char ch;
	int stl;

	stl = strlen(st);
	if (x == RIGHT)
		x = 84 - (stl * cfont.x_size);
	if (x == CENTER)
		x = (84 - (stl * cfont.x_size)) / 2;

	for (int cnt = 0; cnt < stl; cnt++)
		_print_char(*st++, x + (cnt * (cfont.x_size)), y);
}

void Nokia5110Lcd::print(String st, int x, int y) {
	char buf[st.length() + 1];

	st.toCharArray(buf, st.length() + 1);
	print(buf, x, y);
}

void Nokia5110Lcd::printNumI(long num, int x, int y, int length, char filler) {
	char buf[25];
	char st[27];
	boolean neg = false;
	int c = 0, f = 0;

	if (num == 0) {
		if (length != 0) {
			for (c = 0; c < (length - 1); c++)
				st[c] = filler;
			st[c] = 48;
			st[c + 1] = 0;
		} else {
			st[0] = 48;
			st[1] = 0;
		}
	} else {
		if (num < 0) {
			neg = true;
			num = -num;
		}

		while (num > 0) {
			buf[c] = 48 + (num % 10);
			c++;
			num = (num - (num % 10)) / 10;
		}
		buf[c] = 0;

		if (neg) {
			st[0] = 45;
		}

		if (length > (c + neg)) {
			for (int i = 0; i < (length - c - neg); i++) {
				st[i + neg] = filler;
				f++;
			}
		}

		for (int i = 0; i < c; i++) {
			st[i + neg + f] = buf[c - i - 1];
		}
		st[c + neg + f] = 0;

	}

	print(st, x, y);
}

void Nokia5110Lcd::printNumF(double num, byte dec, int x, int y, char divider, int length, char filler) {
	char st[27];
	boolean neg = false;

	if (num < 0)
		neg = true;

	_convert_float(st, num, length, dec);

	if (divider != '.') {
		for (int i = 0; i < sizeof(st); i++)
			if (st[i] == '.')
				st[i] = divider;
	}

	if (filler != ' ') {
		if (neg) {
			st[0] = '-';
			for (int i = 1; i < sizeof(st); i++)
				if ((st[i] == ' ') || (st[i] == '-'))
					st[i] = filler;
		} else {
			for (int i = 0; i < sizeof(st); i++)
				if (st[i] == ' ')
					st[i] = filler;
		}
	}

	print(st, x, y);
}

void Nokia5110Lcd::_print_char(unsigned char c, int x, int y) {
	if ((cfont.y_size % 8) == 0) {
		int font_idx = ((c - cfont.offset) * (cfont.x_size * (cfont.y_size / 8))) + 4;
		for (int rowcnt = 0; rowcnt < (cfont.y_size / 8); rowcnt++) {
			for (int cnt = 0; cnt < cfont.x_size; cnt++) {
				for (int b = 0; b < 8; b++)
					if ((fontbyte(font_idx+cnt+(rowcnt*cfont.x_size)) & (1 << b)) != 0)
						if (cfont.inverted == 0)
							setPixel(x + cnt, y + (rowcnt * 8) + b);
						else
							clrPixel(x + cnt, y + (rowcnt * 8) + b);
					else if (cfont.inverted == 0)
						clrPixel(x + cnt, y + (rowcnt * 8) + b);
					else
						setPixel(x + cnt, y + (rowcnt * 8) + b);
			}
		}
	} else {
		int font_idx = ((c - cfont.offset) * ((cfont.x_size * cfont.y_size / 8))) + 4;
		int cbyte = fontbyte(font_idx);
		int cbit = 7;
		for (int cx = 0; cx < cfont.x_size; cx++) {
			for (int cy = 0; cy < cfont.y_size; cy++) {
				if ((cbyte & (1 << cbit)) != 0)
					if (cfont.inverted == 0)
						setPixel(x + cx, y + cy);
					else
						clrPixel(x + cx, y + cy);
				else if (cfont.inverted == 0)
					clrPixel(x + cx, y + cy);
				else
					setPixel(x + cx, y + cy);
				cbit--;
				if (cbit < 0) {
					cbit = 7;
					font_idx++;
					cbyte = fontbyte(font_idx);
				}
			}
		}
	}
}

void Nokia5110Lcd::setFont(const uint8_t *font) {
	cfont.font = font;
	cfont.x_size = fontbyte(0);
	cfont.y_size = fontbyte(1);
	cfont.offset = fontbyte(2);
	cfont.numchars = fontbyte(3);
	cfont.inverted = 0;
}

void Nokia5110Lcd::drawHLine(int x, int y, int l) {
	int by, bi;

	if ((x >= 0) and (x < 84) and (y >= 0) and (y < 48)) {
		for (int cx = 0; cx < l; cx++) {
			by = ((y / 8) * 84) + x;
			bi = y % 8;

			scrbuf[by + cx] |= (1 << bi);
		}
	}
}

void Nokia5110Lcd::clrHLine(int x, int y, int l) {
	int by, bi;

	if ((x >= 0) and (x < 84) and (y >= 0) and (y < 48)) {
		for (int cx = 0; cx < l; cx++) {
			by = ((y / 8) * 84) + x;
			bi = y % 8;

			scrbuf[by + cx] &= ~(1 << bi);
		}
	}
}

void Nokia5110Lcd::drawVLine(int x, int y, int l) {
	int by, bi;

	if ((x >= 0) and (x < 84) and (y >= 0) and (y < 48)) {
		for (int cy = 0; cy < l; cy++) {
			setPixel(x, y + cy);
		}
	}
}

void Nokia5110Lcd::clrVLine(int x, int y, int l) {
	int by, bi;

	if ((x >= 0) and (x < 84) and (y >= 0) and (y < 48)) {
		for (int cy = 0; cy < l; cy++) {
			clrPixel(x, y + cy);
		}
	}
}

void Nokia5110Lcd::drawLine(int x1, int y1, int x2, int y2) {
	int tmp;
	double delta, tx, ty;
	double m, b, dx, dy;

	if (((x2 - x1) < 0)) {
		tmp = x1;
		x1 = x2;
		x2 = tmp;
		tmp = y1;
		y1 = y2;
		y2 = tmp;
	}
	if (((y2 - y1) < 0)) {
		tmp = x1;
		x1 = x2;
		x2 = tmp;
		tmp = y1;
		y1 = y2;
		y2 = tmp;
	}

	if (y1 == y2) {
		if (x1 > x2) {
			tmp = x1;
			x1 = x2;
			x2 = tmp;
		}
		drawHLine(x1, y1, x2 - x1);
	} else if (x1 == x2) {
		if (y1 > y2) {
			tmp = y1;
			y1 = y2;
			y2 = tmp;
		}
		drawVLine(x1, y1, y2 - y1);
	} else if (abs(x2-x1) > abs(y2 - y1)) {
		delta = (double(y2 - y1) / double(x2 - x1));
		ty = double(y1);
		if (x1 > x2) {
			for (int i = x1; i >= x2; i--) {
				setPixel(i, int(ty + 0.5));
				ty = ty - delta;
			}
		} else {
			for (int i = x1; i <= x2; i++) {
				setPixel(i, int(ty + 0.5));
				ty = ty + delta;
			}
		}
	} else {
		delta = (float(x2 - x1) / float(y2 - y1));
		tx = float(x1);
		if (y1 > y2) {
			for (int i = y2 + 1; i > y1; i--) {
				setPixel(int(tx + 0.5), i);
				tx = tx + delta;
			}
		} else {
			for (int i = y1; i < y2 + 1; i++) {
				setPixel(int(tx + 0.5), i);
				tx = tx + delta;
			}
		}
	}

}

void Nokia5110Lcd::clrLine(int x1, int y1, int x2, int y2) {
	int tmp;
	double delta, tx, ty;
	double m, b, dx, dy;

	if (((x2 - x1) < 0)) {
		tmp = x1;
		x1 = x2;
		x2 = tmp;
		tmp = y1;
		y1 = y2;
		y2 = tmp;
	}
	if (((y2 - y1) < 0)) {
		tmp = x1;
		x1 = x2;
		x2 = tmp;
		tmp = y1;
		y1 = y2;
		y2 = tmp;
	}

	if (y1 == y2) {
		if (x1 > x2) {
			tmp = x1;
			x1 = x2;
			x2 = tmp;
		}
		clrHLine(x1, y1, x2 - x1);
	} else if (x1 == x2) {
		if (y1 > y2) {
			tmp = y1;
			y1 = y2;
			y2 = tmp;
		}
		clrVLine(x1, y1, y2 - y1);
	} else if (abs(x2-x1) > abs(y2 - y1)) {
		delta = (double(y2 - y1) / double(x2 - x1));
		ty = double(y1);
		if (x1 > x2) {
			for (int i = x1; i >= x2; i--) {
				clrPixel(i, int(ty + 0.5));
				ty = ty - delta;
			}
		} else {
			for (int i = x1; i <= x2; i++) {
				clrPixel(i, int(ty + 0.5));
				ty = ty + delta;
			}
		}
	} else {
		delta = (float(x2 - x1) / float(y2 - y1));
		tx = float(x1);
		if (y1 > y2) {
			for (int i = y2 + 1; i > y1; i--) {
				clrPixel(int(tx + 0.5), i);
				tx = tx + delta;
			}
		} else {
			for (int i = y1; i < y2 + 1; i++) {
				clrPixel(int(tx + 0.5), i);
				tx = tx + delta;
			}
		}
	}

}

void Nokia5110Lcd::drawRect(int x1, int y1, int x2, int y2) {
	int tmp;

	if (x1 > x2) {
		tmp = x1;
		x1 = x2;
		x2 = tmp;
	}
	if (y1 > y2) {
		tmp = y1;
		y1 = y2;
		y2 = tmp;
	}

	drawHLine(x1, y1, x2 - x1);
	drawHLine(x1, y2, x2 - x1);
	drawVLine(x1, y1, y2 - y1);
	drawVLine(x2, y1, y2 - y1 + 1);
}

void Nokia5110Lcd::clrRect(int x1, int y1, int x2, int y2) {
	int tmp;

	if (x1 > x2) {
		tmp = x1;
		x1 = x2;
		x2 = tmp;
	}
	if (y1 > y2) {
		tmp = y1;
		y1 = y2;
		y2 = tmp;
	}

	clrHLine(x1, y1, x2 - x1);
	clrHLine(x1, y2, x2 - x1);
	clrVLine(x1, y1, y2 - y1);
	clrVLine(x2, y1, y2 - y1 + 1);
}

void Nokia5110Lcd::drawRoundRect(int x1, int y1, int x2, int y2) {
	int tmp;

	if (x1 > x2) {
		tmp = x1;
		x1 = x2;
		x2 = tmp;
	}
	if (y1 > y2) {
		tmp = y1;
		y1 = y2;
		y2 = tmp;
	}
	if ((x2 - x1) > 4 && (y2 - y1) > 4) {
		setPixel(x1 + 1, y1 + 1);
		setPixel(x2 - 1, y1 + 1);
		setPixel(x1 + 1, y2 - 1);
		setPixel(x2 - 1, y2 - 1);
		drawHLine(x1 + 2, y1, x2 - x1 - 3);
		drawHLine(x1 + 2, y2, x2 - x1 - 3);
		drawVLine(x1, y1 + 2, y2 - y1 - 3);
		drawVLine(x2, y1 + 2, y2 - y1 - 3);
	}
}

void Nokia5110Lcd::clrRoundRect(int x1, int y1, int x2, int y2) {
	int tmp;

	if (x1 > x2) {
		tmp = x1;
		x1 = x2;
		x2 = tmp;
	}
	if (y1 > y2) {
		tmp = y1;
		y1 = y2;
		y2 = tmp;
	}
	if ((x2 - x1) > 4 && (y2 - y1) > 4) {
		clrPixel(x1 + 1, y1 + 1);
		clrPixel(x2 - 1, y1 + 1);
		clrPixel(x1 + 1, y2 - 1);
		clrPixel(x2 - 1, y2 - 1);
		clrHLine(x1 + 2, y1, x2 - x1 - 3);
		clrHLine(x1 + 2, y2, x2 - x1 - 3);
		clrVLine(x1, y1 + 2, y2 - y1 - 3);
		clrVLine(x2, y1 + 2, y2 - y1 - 3);
	}
}

void Nokia5110Lcd::drawCircle(int x, int y, int radius) {
	int f = 1 - radius;
	int ddF_x = 1;
	int ddF_y = -2 * radius;
	int x1 = 0;
	int y1 = radius;
	char ch, cl;

	setPixel(x, y + radius);
	setPixel(x, y - radius);
	setPixel(x + radius, y);
	setPixel(x - radius, y);

	while (x1 < y1) {
		if (f >= 0) {
			y1--;
			ddF_y += 2;
			f += ddF_y;
		}
		x1++;
		ddF_x += 2;
		f += ddF_x;
		setPixel(x + x1, y + y1);
		setPixel(x - x1, y + y1);
		setPixel(x + x1, y - y1);
		setPixel(x - x1, y - y1);
		setPixel(x + y1, y + x1);
		setPixel(x - y1, y + x1);
		setPixel(x + y1, y - x1);
		setPixel(x - y1, y - x1);
	}
}

void Nokia5110Lcd::clrCircle(int x, int y, int radius) {
	int f = 1 - radius;
	int ddF_x = 1;
	int ddF_y = -2 * radius;
	int x1 = 0;
	int y1 = radius;
	char ch, cl;

	clrPixel(x, y + radius);
	clrPixel(x, y - radius);
	clrPixel(x + radius, y);
	clrPixel(x - radius, y);

	while (x1 < y1) {
		if (f >= 0) {
			y1--;
			ddF_y += 2;
			f += ddF_y;
		}
		x1++;
		ddF_x += 2;
		f += ddF_x;
		clrPixel(x + x1, y + y1);
		clrPixel(x - x1, y + y1);
		clrPixel(x + x1, y - y1);
		clrPixel(x - x1, y - y1);
		clrPixel(x + y1, y + x1);
		clrPixel(x - y1, y + x1);
		clrPixel(x + y1, y - x1);
		clrPixel(x - y1, y - x1);
	}
}

/**
 *    @brief    Fill a rectangle completely with one color. Update in subclasses if
 desired!
 @param    x   Top left corner x coordinate
 @param    y   Top left corner y coordinate
 @param    w   Width in pixels
 @param    h   Height in pixels
 *
 */
void Nokia5110Lcd::clrArea(int x, int y, int w, int h) {

	for (uint16_t sor = y; sor < min(h, LCDHEIGHT); sor++) {
		for (uint16_t oszlop = x; oszlop < min(w, LCDWIDTH); oszlop++) {
			clrPixel(oszlop, sor);
		}
	}
}

/**
 *
 */
void Nokia5110Lcd::drawBitmap(int x, int y, uint8_t *bitmap, int sx, int sy) {
	int bit;
	byte data;

	for (int cy = 0; cy < sy; cy++) {
		bit = cy % 8;
		for (int cx = 0; cx < sx; cx++) {
			data = bitmapbyte(cx + ((cy / 8) * sx));
			if ((data & (1 << bit)) > 0)
				setPixel(x + cx, y + cy);
			else
				clrPixel(x + cx, y + cy);
		}
	}
}
