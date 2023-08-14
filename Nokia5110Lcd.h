/*
 * Nokia5110Lcd.h
 *
 *  Created on: 2022. okt. 29.
 *      Author: BT
 */

#ifndef NOKIA5110LCD_H_
#define NOKIA5110LCD_H_

#include <Arduino.h>

#define LCDWIDTH	84
#define LCDHEIGHT	48

#define LEFT 0
#define RIGHT 9999
#define CENTER 9998

#define regtype volatile uint8_t
#define regsize volatile uint8_t


struct _current_font {
	uint8_t *font;
	uint8_t x_size;
	uint8_t y_size;
	uint8_t offset;
	uint8_t numchars;
	uint8_t inverted;
};

class Nokia5110Lcd {
public:
	Nokia5110Lcd(int SCK, int MOSI, int DC, int RST, int CS);
	void init(int contrast = 70);

	void clrScr();
	void update();
	void setContrast(int contrast);
	void enableSleep();
	void disableSleep();
	void fillScr();

	void invert(bool mode);
	void setPixel(uint16_t x, uint16_t y);
	void clrPixel(uint16_t x, uint16_t y);
	void invPixel(uint16_t x, uint16_t y);
	void invertText(bool mode);

	void print(char *st, int x, int y);
	void print(String st, int x, int y);
	void printNumI(long num, int x, int y, int length, char filler = ' ');
	void printNumF(double num, byte dec, int x, int y, char divider, int length, char filler = ' ');

	void setFont(const uint8_t *font);

	void drawHLine(int x, int y, int l);
	void clrHLine(int x, int y, int l);

	void drawVLine(int x, int y, int l);
	void clrVLine(int x, int y, int l);

	void drawLine(int x1, int y1, int x2, int y2);
	void clrLine(int x1, int y1, int x2, int y2);

	void drawRect(int x1, int y1, int x2, int y2);
	void clrRect(int x1, int y1, int x2, int y2);

	//lekerekített élek
	void drawRoundRect(int x1, int y1, int x2, int y2);
	void clrRoundRect(int x1, int y1, int x2, int y2);

	void drawCircle(int x, int y, int radius);
	void clrCircle(int x, int y, int radius);

	void clrArea(int x1, int y1, int x2, int y2);
	void drawBitmap(int x, int y, uint8_t* bitmap, int sx, int sy);

protected:
	uint8_t scrbuf[504];
	regtype *P_SCK, *P_MOSI, *P_DC, *P_RST, *P_CS;
	regsize B_SCK, B_MOSI, B_DC, B_RST, B_CS;
	uint8_t SCK_Pin, RST_Pin;			// Needed for for faster MCUs
	_current_font cfont;
	boolean _sleep = false;
	int _contrast = 70;

	void _convert_float(char *buf, double num, int width, byte prec) {
		dtostrf(num, width, prec, buf);
	}
	void _LCD_Write(unsigned char data, unsigned char mode);
	void _print_char(unsigned char c, int x, int y);

};

#endif /* NOKIA5110LCD_H_ */
