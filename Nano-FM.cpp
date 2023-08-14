#include <Arduino.h>
#include <OneButton.h>
#include <LowPower.h>

#include "Nokia5110Lcd.h"
#include "defaults.h"

//Kütyü üzemállapotok
typedef enum {
	STATE_NORMAL, STATE_INTERNAL, STATE_OFF, STATE_WAKE_UP
} State_t;
volatile State_t currentState = STATE_NORMAL;

////--- Hőmérés -------------------------------------------------------------------------
#define PIN_TEMP_SENSOR 				3		/* ATmega328P PIN:5, D3 a DS18B20 bemenete */
#define DS18B20_TEMP_SENSOR_NDX 		0		/* Dallas DS18B20 hõmérõ szenzor indexe */
#include <OneWire.h>
#define REQUIRESALARMS 					false	/* nem kell a DallasTemperature ALARM supportja */
#define ONEWIRE_SEARCH					0
#define ONEWIRE_CRC						0
#include <DallasTemperature.h>
DallasTemperature ds18B20(new OneWire(PIN_TEMP_SENSOR));

//--- Gombok ----------------------------
#define PIN_MENU_BTN 2					/* ATmega328P PIN:4, D2 a DS18B20 bemenete */
OneButton menuBtn = OneButton(PIN_MENU_BTN, true, true);
OneButton minusBtn = OneButton(A1, true, true);
OneButton plusBtn = OneButton(A2, true, true);

//--- Menü ------------------------------
#define NUMBER_OF_CHAR_ARRAY_ELEMENT(x) (sizeof(x) / sizeof(x[0]))
const char *menuBtnClickOptionsStr[] = { "Frequency", "Seek", "Volume", "BassBoost", "SoftMute", "LCD BLight", "RDS" };
#define MENUBTN_CLICK_FREQUENCY_NDX		0
#define MENUBTN_CLICK_SEEK_NDX			1
#define MENUBTN_CLICK_VOLUME_NDX		2
#define MENUBTN_CLICK_BASSBOOST_NDX		3
#define MENUBTN_CLICK_SOFTMUTE_NDX		4
#define MENUBTN_LCD_BACKLIGHT_NDX		5
#define MENUBTN_LCD_RDS_NDX				6
#define MENUBTN_DEFAULT_INDEX			MENUBTN_CLICK_SEEK_NDX	/* seek a defaut menüpont a bekapcsolást követően */
uint8_t menuBtnCurrentChangeIndex = MENUBTN_DEFAULT_INDEX;

//---LCD Display ------------------------
#define DEGREE_SYMBOL_CODE 				247		/* Az LCD-n a '°' jel kódja */
#define PIN_LCD_BACKLIGHT_LED 			9		/* ATmega328P PIN:15 */
#define PIN_LCD_SCLK					8		/* ATmega328P PIN:14 */
#define PIN_LCD_DIN						7		/* ATmega328P PIN:13 */
#define PIN_LCD_DC						6		/* ATmega328P PIN:12 */
#define PIN_LCD_CS						5		/* ATmega328P PIN:11 */
#define PIN_LCD_RST						4		/* ATmega328P PIN:6 */
Nokia5110Lcd lcd(PIN_LCD_SCLK, PIN_LCD_DIN, PIN_LCD_DC, PIN_LCD_RST, PIN_LCD_CS);

#include "Nokia5110Lcd_Fonts.c"
extern const uint8_t TinyFont[];
extern const uint8_t SmallFont[];

#include "logo.c"
extern const uint8_t Logo[];

//--- LCD háttér ---
//LCD Háttérvilágítás állítás
#include "LcdBackLightAdjuster.h"
#define PIN_PHOTO_SENSOR				A0 		/* ATmega328P PIN:23  */
LcdBackLightAdjuster lcdBackLightAdjuster(PIN_PHOTO_SENSOR, PIN_LCD_BACKLIGHT_LED);

//--- Radio ------------------------------
#include "RDA5807M.h"
RDA5807M radio;
uint16_t _lastFreq; //kikapcsolás előtti freki
RADIO_INFO radioInfo;

//Erősítő PWR PIN
#define PIN_AMP_PWR						10	/* ATmega328P PB2, PIN:16 */

//--- Feszmérés ------------------------------------------------------------------------
//---- Akkumulátor töltöttség mérése
#define INT_REF_VOLTAGE					1.1f													/* 1.1V a belső referencia feszültség */
#define BATT_VOLTAGE_LOW				3.2f													/* Alacsony akkufesz */
#define BATT_VOLTAGE_HIGH				4.2f													/* Teljesen feltötltött akkufesz */

/**
 * Free Memory
 */
extern unsigned int __bss_end;
extern void *__brkval;
int getFreeMemory() {
	int free_memory;

	if ((int) __brkval == 0) {
		free_memory = ((int) &free_memory) - ((int) &__bss_end);
	} else {
		free_memory = ((int) &free_memory) - ((int) __brkval);
	}

	return free_memory;
}

/**
 * Feszmérés
 */
float readVccInVolts() {

	// Read 1.1V reference against AVcc
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA, ADSC))
		;
	long result = ADCL;
	result |= ADCH << 8;
	result = 1126400L / result; // Back-calculate AVcc in mV

	return ((float) result) / 1000.0;
}

//--- Hőmérséklet mérés ----------------------------------------------------------------

/**
 * DS18B20 digitális hőmérő szenzor olvasása
 */
float readTemperature(void) {
	ds18B20.requestTemperaturesByIndex(DS18B20_TEMP_SENSOR_NDX);
	return ds18B20.getTempCByIndex(DS18B20_TEMP_SENSOR_NDX);
}

//--- Loop ----------------------------------------------------------------------------

/**
 * Rádió infók lekérése
 */
void loopRadio() {

	radio.getRadioInfo(&radioInfo);

	//Ha van RDS, akkor azt is feldolgoztatjuk
	if (radioInfo.rdsEnabled && radioInfo.tuned && radioInfo.rdsAvail) {
		radio.ProcessRdsData();
	}
}

/**
 * Kiemelve, hogy a seek alatt is lehessen látni a frekvenciát
 */
void displayFrequency(uint16_t currentFreq) {
	//char tmpBuff[7];
	//sprintf(tmpBuff, "%3d.%1d", currentFreq / 100, (currentFreq % 100) / 10);
	lcd.setFont(MediumNumbers); //big font
	lcd.printNumF(currentFreq / 100.0f, 1, 0, 12, '.', 2);

	lcd.setFont(SmallFont);
	lcd.print(F("MHz"), 65, 21);
}

/**
 *  Seek alatt a frekvencia kijelzése
 */
void displayFrequencyWhileSeeek() {

	//Az előző frekvencia kijelzésének törlése
//	lcd.fillRect(0, 14, LCDWIDTH, 30, WHITE);

	//Frekvencia kiírása
	displayFrequency(radio.getFrequency());

	//megjelenítés
	lcd.update();
}

/**
 * Display normál állapotban
 */
void loopNormalDisplay() {
	lcd.clrScr();

	//Mini font
	lcd.setFont(TinyFont);

// --- Status -----

#define STATUS_ROW				0

	//térerő háromszög
#define RSSI_SIMBOL_HEIGHT 		5
#define MAX_RSSI_SYMBOL_WIDTH 	14
	uint8_t rssiSymBolWidth = map(radioInfo.rssi, 0, RADIO_MAX_RSSI, 0, MAX_RSSI_SYMBOL_WIDTH);
	lcd.drawLine(STATUS_ROW, 4, rssiSymBolWidth, STATUS_ROW); //átfogó
	lcd.drawHLine(STATUS_ROW, 4, rssiSymBolWidth + 1); //alsó vízszintes
	lcd.drawVLine(rssiSymBolWidth, STATUS_ROW, RSSI_SIMBOL_HEIGHT); //jobb függőleges

	//RSSI érték
	lcd.printNumI(radioInfo.rssi, 16, STATUS_ROW, 2);

	//Sztereó szimbólum
	if (radioInfo.stereo) {
		lcd.drawCircle(31, 2, 2);
		lcd.drawCircle(34, 2, 2);
	}

	//RDS felirat
	if (radioInfo.rdsSync) {
		lcd.print(F("RDS"), 45, STATUS_ROW);
	}

	//
	// Batterry mérés eredménye
	//
#define BATT_SYMBOL_LENGTH 15
//	lcd.drawRect(60, 0, 83, 5); //battery blokkja
//	lcd.fillRect(LCDWIDTH - 2, 2, 1, 2, BLACK); //batterry szimbólum "pozitív" vége
//	//Feszmérés
//	byte battery = map(min(readVccInMilliVolts(), BATT_VOLTAGE_HIGH), BATT_VOLTAGE_LOW, BATT_VOLTAGE_HIGH, 0, BATT_SYMBOL_LENGTH - 1);
//	lcd.fillRect(LCDWIDTH - BATT_SYMBOL_LENGTH - 1, 1, battery, 4, BLACK);
	lcd.printNumF(readVccInVolts(), 2, 63, STATUS_ROW, '.', 1);
	lcd.print(F("V"), 80, STATUS_ROW);

// --- Menü -----

	// Mit változtat a rotary encoder? -> kiírjuk
#define MENU_ROW	6
	lcd.print(menuBtnClickOptionsStr[menuBtnCurrentChangeIndex], 0, MENU_ROW);
	switch (menuBtnCurrentChangeIndex) {
	case MENUBTN_CLICK_VOLUME_NDX:	// Volume
		lcd.printNumI(radio.getVolume(), 75, MENU_ROW, 2);
		break;

	case MENUBTN_CLICK_BASSBOOST_NDX: //BasBoost
		lcd.print(radioInfo.bassBoost ? F("On") : F("Off"), 70, MENU_ROW);
		break;

	case MENUBTN_CLICK_SOFTMUTE_NDX: //SoftMute
		lcd.print(radioInfo.softMute ? F("On") : F("Off"), 70, MENU_ROW);
		break;

	case MENUBTN_LCD_BACKLIGHT_NDX:	//LCD BackLight
		lcd.print(lcdBackLightAdjuster.blState ? F("On") : F("Off"), 70, MENU_ROW);
		break;

	case MENUBTN_LCD_RDS_NDX: //RDS
		lcd.print(radioInfo.rdsEnabled ? F("On") : F("Off"), 70, MENU_ROW);
		break;
	}

	//
	// frekvencia kijelzése
	//
	displayFrequency(radio.getFrequency());

	//
	// RDS infók
	//
#define RDS_INFO_STATION_AND_TIME_LINE_Y 30
	//Service name
	lcd.setFont(SmallFont);

	if (radio.StationName[0] != NULL) {
		lcd.print(radio.StationName, 0, RDS_INFO_STATION_AND_TIME_LINE_Y);
	}

	//RDS time
	if (radio.CTtime[0] != NULL) {
		lcd.print(radio.CTtime, 53, RDS_INFO_STATION_AND_TIME_LINE_Y);
	}

#define RDS_INFO_TEXT_LINE_Y 40
#define RDS_SCROLL_POSITION_BEGIN -13
	static int8_t rdsTextScrollPosition = RDS_SCROLL_POSITION_BEGIN;
	//RDS text scroll
	if (radio.RDSText[0] != NULL) {
		for (int8_t i = rdsTextScrollPosition; i < rdsTextScrollPosition + (-1) * RDS_SCROLL_POSITION_BEGIN + 1; i++) {
			if ((i >= strlen(radio.RDSText)) || (i < 0)) {
				lcd.print(" ", 0, RDS_INFO_TEXT_LINE_Y);
			} else {
				lcd.print(&radio.RDSText[i], 0, RDS_INFO_TEXT_LINE_Y);
			}
		}
		rdsTextScrollPosition++;
		if ((rdsTextScrollPosition >= strlen(radio.RDSText)) && (rdsTextScrollPosition > 0)) {
			rdsTextScrollPosition = RDS_SCROLL_POSITION_BEGIN;
		}
	}

	lcd.update();
}

/**
 * Kikapcsolt állapotban a display kezelése
 */
void loopStandby() {

#define STANDBY_ROW_BEGIN	22
	//Az előző fesz és hőmérséklet kijelzésének törlése
	lcd.clrArea(0, STANDBY_ROW_BEGIN, LCDWIDTH, LCDHEIGHT);

	//--- Hőmérséklet
	lcd.setFont(MediumNumbers); //big font
	lcd.printNumF(readTemperature(), 2, 10, STANDBY_ROW_BEGIN, '.', 3);
	//A ° jel kiírása
	lcd.setFont(SmallFont);
	lcd.print(F("C"), 75, 30);

	//--- Akku fesz
#define VOLRAGE_ROW 43
	lcd.setFont(TinyFont); //Mini font
	lcd.print(F("Batterry: "), 12, VOLRAGE_ROW);
	lcd.printNumF(readVccInVolts(), 3, 50, VOLRAGE_ROW, '.', 1);
	lcd.print(F("V"), 74, VOLRAGE_ROW);

	lcd.update();
}

/**
 * Display a belső mérés állapotban
 */
void loopInternalDisplay() {

	lcd.clrScr();
	lcd.setFont(SmallFont);

	lcd.invertText(true);
	lcd.print(F("-INTERN STATE-"), 0, 0);
	lcd.invertText(false);

	lcd.setFont(TinyFont);
	lcd.print(F("Batt: "), 0, 10);
	lcd.printNumF(readVccInVolts(), 2, 35, 10, '.', 2);
	lcd.print(F("[V]"), 65, 10);

	lcd.print(F("Temp: "), 0, 16);
	lcd.printNumF(readTemperature(), 2, 35, 16, '.', 2);
	lcd.print(F("[C]"), 65, 16);

	lcd.print(F("FMem: "), 0, 22);
	lcd.printNumI(getFreeMemory(), 35, 22, 3);
	lcd.print(F("[B]"), 65, 22);

	lcd.print(F("RSSI: "), 0, 28);
	radio.getRadioInfo(&radioInfo);
	lcd.printNumI(radioInfo.rssi, 35, 28, 2);
	lcd.print(F("[-]"), 65, 28);

	lcd.print(__DATE__, 0, 42);
	lcd.print(__TIME__, 52, 42);

	lcd.update();
}

//--- On/Off ---------------------------------------------------------------------------
/**
 * Rendszer bekapcsolása
 */
void systemSwithcOn() {

	//Leszállunk a megszakításról
	detachInterrupt(digitalPinToInterrupt(PIN_MENU_BTN));

	//LCD init
	lcd.init(30);


	//Rádió Be
	radio.powerUp();
	radio.setFrequency(_lastFreq);

	//Seek változtatás aktív
	menuBtnCurrentChangeIndex = MENUBTN_DEFAULT_INDEX;

	// LCD LED állítgatás
	lcdBackLightAdjuster.restore();

	//Erősítő be
	digitalWrite(PIN_AMP_PWR, HIGH);

	//Buttonok reset
	menuBtn.reset();
	minusBtn.reset();
	plusBtn.reset();

	//Bekapcs
	currentState = STATE_NORMAL;

}

/**
 * PowerDown módban megszakítás kezelése
 *
 * - Ha Standby állapotban megnyomták a gombot
 * - Ha a lemerült akkumulátor miatt kikapcsolt állapotban voltunk és megnomyták a gombot
 */
volatile long lastPowerDownWakeUpIsrTime = millis();
void powerDownWakeUpIsr() {

	if (millis() - lastPowerDownWakeUpIsrTime > 100) {

#ifdef SERIAL_DEBUG
		Serial.println("powerDownWakeUpIsr called!");
#endif

		lastPowerDownWakeUpIsrTime = millis();
		detachInterrupt(digitalPinToInterrupt(PIN_MENU_BTN));
		currentState = STATE_WAKE_UP;
	}
}

/**
 *  Ráköltözünk gomb megnyomás megszakításra
 */
void hookInterrupt() {

	//beállítjuk, hogy stadby állapotban vagyunk
	currentState = STATE_OFF;

	//Ráköltözünk gomb megnyomás megszakításra
	lastPowerDownWakeUpIsrTime = millis();
	attachInterrupt(digitalPinToInterrupt(PIN_MENU_BTN), powerDownWakeUpIsr, LOW);

	//kíírjuk az adatokat
	loopStandby();
}

/**
 * Rendszer kikapcsolása
 */
void systemSwithcOff() {

	//konfig mentése

	// LCD LED Off
	lcdBackLightAdjuster.off();

	//rádió lecsukása
	_lastFreq = radio.getFrequency();
	radio.powerDown();
	radio.clearRDS();


	//LCD init
	lcd.init(30);

	lcd.clrScr();
	lcd.drawBitmap(10, 0, (uint8_t*) Logo, 60, 8);
	//bitmap piszkok törlése
	lcd.clrPixel(10, 3);
	lcd.clrPixel(12, 3);
	lcd.clrPixel(10, 5);
	lcd.clrPixel(10, 4);
	lcd.update();


	lcd.setFont(SmallFont);
	lcd.print(F("Radio"), 5, 12);
	lcd.print(F(RADIO_VERSION), 45, 12);
	lcd.update();

	//Erősítő ki
	digitalWrite(PIN_AMP_PWR, LOW);

	//-----------------

	//Ráköltözünk gomb megnyomás megszakításra
	hookInterrupt();
}

/**
 * Alacsony akkufeszültés vizsgálata
 */
void loopCheckLowBatterry() {

	//Voltban kifejezve
	float currBattVoltage = readVccInVolts();

	//Akku feszültég még OK?
	if (currBattVoltage >= BATT_VOLTAGE_LOW) {
		return; //akkufesz OK
	}

	// Alacsony az akkufesz!!!

	//Minden kikapcsolása
	systemSwithcOff();

	//Kiírjuk a figyelmeztető üzenetet
	lcd.clrScr();
	lcd.setFont(SmallFont);
	lcd.invertText(true);
	lcd.print(F(" LOW BATTERRY "), CENTER, 0);
	lcd.print(F("! VOLTAGE !"), CENTER, 10);
	lcd.invertText(false);

	lcd.setFont(MediumNumbers); //big font
	lcd.printNumF(currBattVoltage, 2, 15, 25, '.', 2);
	lcd.setFont(SmallFont);
	lcd.print(F("V"), 70, 35);

	lcd.update();
	//lcd.enableSleep();

	//-----------------

	//
	// ---- PowerDown !!
	//
	LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

//---- Gombok ----------------------------------------------------------------------------------------------------------------------
/**
 * Menü click
 */
void handleMenuBtnClick() {

#ifdef SERIAL_DEBUG
	Serial.println("handleMenuBtnClick");
#endif

	if (currentState == STATE_OFF) {
		systemSwithcOn();

	} else if (currentState == STATE_INTERNAL) { //ha belső módban voltunk, akkor visszalépünk normál módba
		currentState = STATE_NORMAL;

	} else {
		//A Menü változtatása
		menuBtnCurrentChangeIndex++;
		if (menuBtnCurrentChangeIndex == NUMBER_OF_CHAR_ARRAY_ELEMENT(menuBtnClickOptionsStr)) {
			menuBtnCurrentChangeIndex = 0;
		}
	}
}

/** --------------------------------------------------------
 *
 * Menü double click
 */
void handleMenuBtnDoubleClick() {
	//Két klikkre belső monitor módba lépünk
	if (currentState == STATE_NORMAL) {
		currentState = STATE_INTERNAL;
	}
}
/**
 * Menü long click -> Kikapcsolás
 */
void handleMenuBtnLongClick() {
	if (currentState != STATE_OFF) {
		systemSwithcOff();
	}
}

/** -------------------------------------------------------
 *
 * Megnyomták a '-' vagy a '+' gombot
 *
 */
void handlePlusMinusBtnClick(bool isPlusClicked) {

	switch (menuBtnCurrentChangeIndex) {
	case MENUBTN_CLICK_FREQUENCY_NDX:
		radio.clearRDS();
		if (isPlusClicked) {
			radio.setFrequencyUp();
		} else {
			radio.setFrequencyDown();
		}
		break;

	case MENUBTN_CLICK_SEEK_NDX:
		radio.clearRDS();
		radio.seek(RDA_SEEK_WRAP, isPlusClicked ? RDA_SEEK_UP : RDA_SEEK_DOWN, displayFrequencyWhileSeeek);
		break;

	case MENUBTN_CLICK_VOLUME_NDX:
		if (isPlusClicked) {
			radio.setVolumeUp();
		} else {
			radio.setVolumeDown();
		}
		break;

	case MENUBTN_CLICK_BASSBOOST_NDX:
		radio.setBass(isPlusClicked);
		break;

	case MENUBTN_CLICK_SOFTMUTE_NDX:
		radio.setSoftmute(isPlusClicked);
		break;

	case MENUBTN_LCD_BACKLIGHT_NDX:
		if (isPlusClicked) {
			if (!lcdBackLightAdjuster.blState) {
				lcdBackLightAdjuster.on();
			}
		} else {
			if (lcdBackLightAdjuster.blState) {
				lcdBackLightAdjuster.off();
			}
		}
		break;

	case MENUBTN_LCD_RDS_NDX:
		radio.clearRDS();
		radio.setRDS(isPlusClicked);
		break;
	}
}

/**
 * '-' Click
 */
void handleMinusBtnClick() {

	if (currentState != STATE_NORMAL) {
		return;
	}

	handlePlusMinusBtnClick(false);
}
/**
 * '+' Click
 */
void handlePlusBtnClick() {

	if (currentState != STATE_NORMAL) {
		return;
	}

	handlePlusMinusBtnClick(true);
}

/** ---------------------------------------------
 *
 * Nyomva tartják a'-' vagy a '+' gombot
 * csak a a menü a frekin áll
 */
void handlePlusMinusBtnDuringLongClick(bool isPlusPushed) {

	if (isPlusPushed) {
		radio.setFrequencyUp();
	} else {
		radio.setFrequencyDown();
	}
}

/**
 * Hosszan nyomják a '-' gombot
 */
void handleMinusBtnDuringLongClick() {

	if (menuBtnCurrentChangeIndex != MENUBTN_CLICK_FREQUENCY_NDX) {
		return;
	}

	handlePlusMinusBtnDuringLongClick(false);
}
/**
 * Hosszan nyomják a '+' gombot
 */
void handlePlusBtnDuringLongClick() {

	if (menuBtnCurrentChangeIndex != MENUBTN_CLICK_FREQUENCY_NDX) {
		return;
	}

	handlePlusMinusBtnDuringLongClick(true);
}

/**
 *
 */
void setup() {
#ifdef SERIAL_DEBUG
	Serial.begin(SERIAL_BAUD_RATE);
#endif

//--- Buttons init
	menuBtn.attachClick(handleMenuBtnClick);
	menuBtn.attachDoubleClick(handleMenuBtnDoubleClick);
	menuBtn.attachLongPressStart(handleMenuBtnLongClick);

	minusBtn.attachClick(handleMinusBtnClick);
	minusBtn.attachDuringLongPress(handleMinusBtnDuringLongClick);

	plusBtn.attachClick(handlePlusBtnClick);
	minusBtn.attachDuringLongPress(handlePlusBtnDuringLongClick);

//--- LCD init
	lcd.init(30);

//--- Radio init
	radio.init();
	radio.setBand(RDA_FM_BAND_USA_EU);
	radio.setSpace(RDA_FM_SPACE_100);
	radio.setFmDeemphasis(RDA_FM_DEEM_50);
	radio.setFrequency(RADIO_DEFAUL_FREQEUNCY); //Petőfi
	radio.setVolume(RADIO_DEFAUL_VOLUME);
	radio.setBass(true);
	radio.setSoftmute(true);
	radio.setMono(false); // Force stereo
	radio.setSeekThreshold(4);
	radio.setRDS(true);
	radio.setRdsFifo(true);

//--- LCD háttérvilágítás
	lcdBackLightAdjuster.init();

//--- Erősítő PWR mód
	pinMode(PIN_AMP_PWR, OUTPUT);

//--- Hőmérő begin
	ds18B20.begin();
	//A funkció a DS18B20 belső ADC-jének felbontását 9, 10, 11 vagy 12 bitre állítja,
	// amelyek 0,5 °C, 0,25 °C, 0,125 °C és 0,0625 °C-nak felelnek meg.
	ds18B20.setResolution(11);

//--- Kikapcsolás
	systemSwithcOff();
}

/**
 *
 */
void loop() {
	static long lastRefresh = millis();

	if (currentState == STATE_NORMAL || currentState == STATE_INTERNAL) {
		menuBtn.tick();
		minusBtn.tick();
		plusBtn.tick();
	}

	switch (currentState) {

	case STATE_NORMAL:	//Normál állapot
		if ((millis() - lastRefresh) >= 700L) {
			loopRadio();
			loopNormalDisplay();
			lastRefresh = millis();
		}
		lcdBackLightAdjuster.adjust();
		break;

	case STATE_INTERNAL: //Belső mérés állapot
		if ((millis() - lastRefresh) >= 3000L) {
			loopInternalDisplay();
			lastRefresh = millis();
		}
		lcdBackLightAdjuster.adjust();
		break;

	case STATE_WAKE_UP:
		systemSwithcOn();
		menuBtnCurrentChangeIndex = MENUBTN_CLICK_FREQUENCY_NDX;
		break;

	default: //Off állapot
		loopStandby();
		LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
		break;
	}

	//Akkufesz mérése 10mp-enként
	static long lastBatterryCheck = 0L;
	if ((millis() - lastBatterryCheck) >= 10000L) {
		//Alacsony feszülség esetén kiírjuk a képernyőre a figyelmeztető üzenetet és kikapcsolunk
		loopCheckLowBatterry();
		lastBatterryCheck = millis();
	}
}
