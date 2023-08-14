/*
 * defaults.h
 *
 *  Created on: 2022. okt. 16.
 *      Author: BT
 */

#ifndef DEFAULTS_H_
#define DEFAULTS_H_

//#define SERIAL_DEBUG
#ifdef SERIAL_DEBUG
#define SERIAL_BAUD_RATE 	115200
#define RDS_SERIAL_DEBUG
#endif



#define RADIO_VERSION	"0.0.9"


//8950 - Music FM
//9390 - Petõfi rádió
//10330 - Retro rádió
#define RADIO_DEFAUL_FREQEUNCY 		9390
#define RADIO_DEFAUL_RDS			true
#define RADIO_DEFAUL_VOLUME 		1
#define RADIO_DEFAUL_BASSBOOST		true
#define RADIO_DEFAUL_SOFTMUTE		true

//LCD
#define LCD_DEFAULT_BACKLIGHT		true;


#endif /* DEFAULTS_H_ */
