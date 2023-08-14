/*
 * RDA5807M.cpp
 *
 *  Created on: 2022. okt. 25.
 *      Author: BT
 */
#include <Arduino.h>
#include <Wire.h>
#include "RDA5807M.h"

/**
 * @brief Sets a given value to a specific device register
 *
 * @see RDA5807M - SINGLE-CHIP BROADCAST FMRADIO TUNER; pages 5, 9, 10 and 11.
 * @see rda_reg02, rda_reg03, rda_reg04, rda_reg05, rda_reg06, rda_reg07
 *
 * @param regAddr    register number (valid values is between 0x02 and 0x07)
 * @param value  the unsigned 16 bits word value (see rda_rec0x data types)
 */
void RDA5807M::setRegister(uint8_t regAddr, uint16_t value) {

	word16_to_bytes aux;

	//csak a 2...7 regiszterek az írhatóak
	if (regAddr < 0x02 || regAddr > 0x07)
		return;

	Wire.beginTransmission(I2C_ADDR_DIRECT_ACCESS);

	Wire.write(regAddr);

	aux.raw = value;
	Wire.write(aux.refined.highByte);
	Wire.write(aux.refined.lowByte);

	Wire.endTransmission();
	delayMicroseconds(3000); // Check
}

/**
 * init
 */
void RDA5807M::init() {
	Wire.begin();
	delay(1);

	reg02.raw = 0x00;
	reg02.refined.DHIZ = 1;    // Normal operation
	reg02.refined.DMUTE = 1;   // Normal operation
	reg02.refined.MONO = 0;    // Force strereo
	reg02.refined.BASS = 1;		//Bass boost on
	reg02.refined.NON_CALIBRATE = 0; //RCLK mindig van
	reg02.refined.RCLK_DIRECT_IN = OSCILLATOR_TYPE_CRYSTAL; 	// Crystal 32.768kHz
	reg02.refined.SEEK = 0;
	reg02.refined.SKMODE = RDA_SEEK_WRAP; //a freki határok elérése után folytassa
	reg02.refined.CLK_MODE = RDA_CLOCK_32K;  // 32.768kHz Kristály
	reg02.refined.RDS_EN = 1;  // RDS enable
	reg02.refined.NEW_METHOD = 0; //new demodulator method
	reg02.refined.SOFT_RESET = 0; //nincs soft reset
	reg02.refined.ENABLE = 1; //Power up
	setRegister(REG02, reg02.raw);

	reg03.raw = 0x00;
	reg03.refined.CHAN = 0; //0. csatorna
	reg03.refined.TUNE = 0; //nem hangolunk
	reg03.refined.BAND = RDA_FM_BAND_USA_EU; //87–108 MHz (US/Europe)
	reg03.refined.SPACE = RDA_FM_SPACE_100; //100kHz
	reg03.refined.DIRECT_MODE = 0; //csak teszre (?)
	setRegister(REG03, reg03.raw);

	reg04.raw = 0x00;
	reg04.refined.DE = RDA_FM_DEEM_50; // 50 μs. Used in Europe, Australia, Japan.
	reg04.refined.SOFTMUTE_EN = 1; //Soft mute
	reg04.refined.AFCD = 0; //AFC bekapcs
	setRegister(REG04, reg04.raw);

	reg05.raw = 0x00;
	reg05.refined.INT_MODE = 0; //5msec interrupt
	reg05.refined.LNA_PORT_SEL = 2; 	//??
	reg05.refined.LNA_ICSEL_BIT = 0;	//??
	reg05.refined.SEEKTH = 8;  // 0b1000 -> seek SNR treshold value
	reg05.refined.VOLUME = 1;	//volume
	setRegister(REG05, reg05.raw);
}

/**
 * @ingroup GA03
 * @brief Powers the receiver on
 */
void RDA5807M::powerUp() {
	reg02.refined.SEEK = 0;
	reg02.refined.ENABLE = 1; //Power Up
	setRegister(REG02, reg02.raw);
	clearRDS();
}

/**
 * @ingroup GA03
 * @brief Power the receiver off
 */
void RDA5807M::powerDown() {
	reg02.refined.SEEK = 0;
	reg02.refined.ENABLE = 0;
	setRegister(REG02, reg02.raw);
}

/**
 * @ingroup GA03
 * @brief Sets the FM band. See table below.
 *
 * FM band table
 *
 * | Value | Description                 |
 * | ----- | --------------------------- |
 * | 00    | 87–108 MHz (US/Europe)      |
 * | 01    | 76–91 MHz (Japan)           |
 * | 10    | 76–108 MHz (world wide)     |
 * | 11    | 65 –76 MHz (East Europe) or 50-65MHz (see bit 9 of gegister 0x06) |
 *
 * @param band FM band index. See table above.
 */
void RDA5807M::setBand(uint8_t band) {
	reg03.refined.BAND = band; // Adjusted by anonimous developer
	setRegister(REG03, reg03.raw);
}

/**
 * @ingroup GA03
 * @brief Sets the FM channel space.
 *
 * Channel space table
 *
 * | Value | Description |
 * | ----- | ----------- |
 * | 00    | 100KHz      |
 * | 01    | 200KHz      |
 * | 10    | 50KHz       |
 * | 11    | 25KHz       |
 *
 * @param space FM channel space. See table above.
 */
void RDA5807M::setSpace(uint8_t space) {
	reg03.refined.SPACE = space;
	setRegister(REG03, reg03.raw);
}

/**
 * @ingroup GA03
 * @brief Sets De-emphasis.
 * @details 75 μs. Used in USA (default); 50 μs. Used in Europe, Australia, Japan.
 *
 * @param de  0 = 75 μs; 1 = 50 μs
 */
void RDA5807M::setFmDeemphasis(uint8_t de) {
	reg04.refined.DE = de;
	setRegister(REG04, reg04.raw);
}

/**
 * @ingroup GA03
 * @brief Waits for Seek or Tune finish
 */
void RDA5807M::waitAndFinishTune() {
	do {
		getStatusRegister(REG0A);
	} while (reg0a->refined.STC == 0);
}

/**
 * @ingroup GA03
 * @brief Sets the channel
 * @details This method tunes the rteceiver in a given channel.
 * @details The channel can be calculated by using the follow formula
 * @details channel = (desired frequency - start band frequency) / space channel in use / 10.0);
 *
 * @see setFrequency, setBand, setSpace
 * @see RDA5807M - SINGLE-CHIP BROADCAST FM RADIO TUNER - Rev.1.1–Aug.2015; pages 9 and 12.
 *
 * @param channel
 */
void RDA5807M::setChannel(uint16_t channel) {
	reg03.refined.CHAN = channel;
	reg03.refined.TUNE = 1;
	reg03.refined.DIRECT_MODE = 0;
	setRegister(REG03, reg03.raw);
	waitAndFinishTune();
}

/**
 * @ingroup GA03
 * @brief Sets the frequency
 * @param frequency
 */
void RDA5807M::setFrequency(uint16_t frequency) {
	uint16_t channel = (frequency - this->startBand[reg03.refined.BAND]) / (this->fmSpace[reg03.refined.SPACE] / 10.0);
	setChannel(channel);
	this->currentFrequency = frequency;
}

/**
 * @ingroup GA03
 * @brief Gets the current channel stored in 0x0A status register.
 *
 * @see setChannel, setFrequency, setBand, setSpace
 * @see RDA5807M - SINGLE-CHIP BROADCAST FM RADIO TUNER - Rev.1.1–Aug.2015; pages 9 and 12.
 *
 * @return uint16_t current channel value
 */
uint16_t RDA5807M::getRealChannel() {
	getStatusRegister(REG0A);
	return reg0a->refined.READCHAN;
}

///**
// * @ingroup GA03
// * @brief Gets the current frequency bases on the current channel.
// * @details The current channel is stored in the 0x0A register. This value is updated after a tune or seek operation.
// * @details The current frequency can be calculated by the formula below
// *
// * | Band   | Formula |
// * | ------ | ------- |
// * |    0   | Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 87.0 MHz |
// * | 1 or 2 | Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 76.0 MHz |
// * |    3   | Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 65.0 MHz |
// *
// * @see setChannel, setFrequency, setBand, setSpace
// * @see RDA5807M - SINGLE-CHIP BROADCAST FM RADIO TUNER - Rev.1.1–Aug.2015; pages 9 and 12.
// * @return uint16_t
// */
uint16_t RDA5807M::getRealFrequency() {
	//return getRealChannel() * (this->fmSpace[reg03.refined.SPACE] / 10.0) + this->startBand[reg03.refined.BAND];
	return this->getRealFrequency(this->getRealChannel());
}

/**
 * @ingroup GA03
 * @brief Gets the current frequency bases on the current channel.
 * @details The current channel is stored in the 0x0A register. This value is updated after a tune or seek operation.
 * @details The current frequency can be calculated by the formula below
 *
 * | Band   | Formula |
 * | ------ | ------- |
 * |    0   | Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 87.0 MHz |
 * | 1 or 2 | Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 76.0 MHz |
 * |    3   | Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 65.0 MHz |
 *
 * @see setChannel, setFrequency, setBand, setSpace
 * @see RDA5807M - SINGLE-CHIP BROADCAST FM RADIO TUNER - Rev.1.1–Aug.2015; pages 9 and 12.
 * @return uint16_t
 */
uint16_t RDA5807M::getRealFrequency(uint16_t realChannel) {
	return realChannel * (this->fmSpace[reg03.refined.SPACE] / 10.0) + this->startBand[reg03.refined.BAND];
}

/**
 * @ingroup GA03
 * @brief Increments the current frequency
 * @details The increment uses the band space as step. See array: uint16_t fmSpace[4] = {100/10, 200/10, 50/10, 25/10};
 */
void RDA5807M::setFrequencyUp() {
	if (this->currentFrequency < this->endBand[reg03.refined.BAND])
		this->currentFrequency += (this->fmSpace[reg03.refined.SPACE] / 10.0);
	else
		this->currentFrequency = this->startBand[reg03.refined.BAND];

	setFrequency(this->currentFrequency);
}

/**
 * @ingroup GA03
 * @brief Decrements the current frequency
 * @details The drecrement uses the band space as step. See array: uint16_t fmSpace[4] = {20, 10, 5, 1};
 */
void RDA5807M::setFrequencyDown() {
	if (this->currentFrequency > startBand[reg03.refined.BAND])
		this->currentFrequency -= (fmSpace[reg03.refined.SPACE] / 10.0);
	else
		this->currentFrequency = endBand[reg03.refined.BAND];

	setFrequency(this->currentFrequency);
}

/**
 * @ingroup GA03
 * @brief Seek function
 *
 * @param seek_mode  if 0, wrap at the upper or lower band limit and continue seeking; 1 = stop seeking at the upper or lower band limit
 * @param direction  if 0, seek down; if 1, seek up.
 */
void RDA5807M::seek(uint8_t seek_mode, uint8_t direction) {
	reg02.refined.SEEK = 1;
	reg02.refined.SKMODE = seek_mode;
	reg02.refined.SEEKUP = direction;
	setRegister(REG02, reg02.raw);
}

/**
 * @ingroup GA03
 * @brief Seek function
 * @details Seeks a station up or down.
 * @details Seek up or down a station and call a function defined by the user to show the frequency during the seek process.
 * @details Seek begins at the current channel, and goes in the direction specified with the SEEKUP bit. Seek operation stops when a channel is qualified as valid according to the seek parameters, the entire band has been searched (SKMODE = 0), or the upper or lower band limit has been reached (SKMODE = 1).
 * @details The STC bit is set high when the seek operation completes and/or the SF/BL bit is set high if the seek operation was unable to find a channel qualified as valid according to the seek parameters. The STC and SF/BL bits must be set low by setting the SEEK bit low before the next seek or tune may begin.
 * @details The SEEK bit is set low and the STC bit is set high when the seek operation completes.
 * @details It is important to say you have to implement a show frequency function. This function have to get the frequency via getFrequency function.
 * @details Example:
 * @code
 *
 * SI470X rx;
 *
 * void showFrequency() {
 *    uint16_t freq = rx.getFrequency();
 *    Serial.print(freq);
 *    Serial.println("MHz ");
 * }
 *
 * void loop() {
 *  .
 *  .
 *      rx.seek(SI470X_SEEK_WRAP, SI470X_SEEK_UP, showFrequency); // Seek Up
 *  .
 *  .
 * }
 * @endcode
 * @param seek_mode  Seek Mode; 0 = Wrap at the upper or lower band limit and continue seeking (default); 1 = Stop seeking at the upper or lower band limit.
 * @param direction  Seek Direction; 0 = Seek down (default); 1 = Seek up.
 * @param showFunc  function that you have to implement to show the frequency during the seeking process. Set NULL if you do not want to show the progress.
 */
void RDA5807M::seek(uint8_t seek_mode, uint8_t direction, void (*showFunc)()) {
	getStatusRegister(REG0A);
	do {
		reg02.refined.SEEK = 1;
		reg02.refined.SKMODE = seek_mode;
		reg02.refined.SEEKUP = direction;
		setRegister(REG02, reg02.raw);

//		//Olvassuk a 0A regisztert az STC (és a READCHAN) miatt
//		delay(10);
//		getStatusRegister(REG0A);
//
//		if (showFunc != NULL) {
//			// Kiszámítjuk a frekvenciát az aktuális csatorna számából
//			this->currentFrequency = this->getRealFrequency(reg0a->refined.READCHAN);
//			showFunc();
//		}

		this->currentFrequency = getRealFrequency(); // gets the current seek frequency
		if (showFunc != NULL) {
			showFunc();
		}
		delay(10);
		getStatusRegister(REG0A);

	} while (reg0a->refined.STC == 0);

	//A végére még egyszer ránézünk
	waitAndFinishTune();
	setFrequency(getRealFrequency(reg0a->refined.READCHAN)); // Fixes station found.
}

//------------------
/**
 * @ingroup GA03
 * @brief Gets the register content of a given status register (from 0x0A to 0x0F)
 * @details Useful when you need just a specific status register content.
 * @details This methos update the first element of the shadowStatusRegisters linked to the register
 * @return rdax_reg0a the reference to current value of the 0x0A register.
 */
void* RDA5807M::getStatusRegister(uint8_t reg) {
	word16_to_bytes aux;

	if (reg < 0x0A || reg > 0x0F)
		return NULL;  // Maybe not necessary.

	Wire.beginTransmission(I2C_ADDR_DIRECT_ACCESS);
	Wire.write(reg);
	Wire.endTransmission(false);

	Wire.requestFrom(I2C_ADDR_DIRECT_ACCESS, 2); // reading 0x0A register
	delayMicroseconds(250);
	aux.refined.highByte = Wire.read();
	aux.refined.lowByte = Wire.read();
	Wire.endTransmission(true);

	shadowStatusRegisters[reg - 0x0A] = aux.raw;

	return &shadowStatusRegisters[reg - 0x0A];
}
/**
 *
 */
void RDA5807M::getStatusRegisters() {
	word16_to_bytes aux;
	int i;

	Wire.requestFrom(I2C_ADDR_FULL_ACCESS, 12); // This call starts reading from 0x0A register
	for (i = 0; i < 6; i++) {
		aux.refined.highByte = Wire.read();
		aux.refined.lowByte = Wire.read();
		shadowStatusRegisters[i] = aux.raw;
	}
	Wire.endTransmission();
}

/**
 * @ingroup GA03
 * @brief Sets the audio volume level
 *
 * @param value
 */
void RDA5807M::setVolume(uint8_t value) {
	if (value > RDA_MAX_VOLUME)
		value = RDA_MAX_VOLUME;

	reg05.refined.VOLUME = value;
	setRegister(REG05, reg05.raw);
}

/**
 * @ingroup GA03
 * @brief Increments the audio volume
 *
 */
void RDA5807M::setVolumeUp() {
	if (this->currentVolume < RDA_MAX_VOLUME) {
		this->currentVolume++;
		setVolume(this->currentVolume);
	}
}

/**
 * @ingroup GA03
 * @brief Decrements the audio volume
 *
 */
void RDA5807M::setVolumeDown() {
	if (this->currentVolume > RDA_MIN_VOLUME) {
		this->currentVolume--;
		setVolume(this->currentVolume);
	}
}

/**
 * @ingroup GA03
 * @brief Sets Bass Boost
 *
 * @param value FALSE = Disable; TRUE = Enable
 */
void RDA5807M::setBass(bool value) {
	reg02.refined.SEEK = 0;
	reg02.refined.BASS = value;
	setRegister(REG02, reg02.raw);
}

/**
 * @ingroup GA03
 * @brief Sets Soft Mute Enable or disable
 * @param value true = enable; false=disable
 */
void RDA5807M::setSoftmute(bool value) {
	reg04.refined.SOFTMUTE_EN = value;
	setRegister(REG04, reg04.raw);
}

/**
 * @ingroup GA03
 * @brief Sets audio Mono or stereo
 *
 * @param value TRUE = Mono; FALSE force stereo
 */
void RDA5807M::setMono(bool value) {
	reg02.refined.SEEK = 0;
	reg02.refined.MONO = value;
	setRegister(REG02, reg02.raw);
}

/**
 * @ingroup GA04
 * @brief Sets the RDS operation
 * @details Enable or Disable the RDS
 *
 * @param true = turns the RDS ON; false  = turns the RDS OFF
 */
void RDA5807M::setRDS(bool value) {
	reg02.refined.SEEK = 0;
	reg02.refined.RDS_EN = value;
	setRegister(REG02, reg02.raw);
}

//--- RDS -------------------------------------------------------------------------------

/**
 * @ingroup GA04
 * @brief Sets RDS fifo mode enable
 *
 * @param value  If true, it makes the the fifo mode enable.
 * @return true  or false
 */
void RDA5807M::setRdsFifo(bool value) {
	reg04.refined.RDS_FIFO_EN = value;
	setRegister(REG04, reg04.raw);
}

/**
 * @ingroup GA04
 * @brief Clear RDS fifo
 *
 * @param value  If true, it makes the the fifo mode enable.
 * @return true  or false
 */
void RDA5807M::clearRdsFifo() {
	reg04.refined.RDS_FIFO_CLR = 1;
	setRegister(REG04, reg04.raw);
}

/**
 * @ingroup GA03
 * @brief Sets RSSI Seek Threshold
 * @param  value
 */
void RDA5807M::setSeekThreshold(uint8_t value) {
	reg05.refined.SEEKTH = value;
	setRegister(REG05, reg05.raw);
}

/**
 *
 */
void RDA5807M::clearRDS() {
	strcpy(StationName, "        ");
	strcpy(PSName, "        ");
	strcpy(PSName1, "        ");
	strcpy(PSName2, "        ");
	memset(RDSText, '\0', sizeof(RDSText));
	memset(RDSTxt, '\0', sizeof(RDSTxt));
	lastTextIDX = 0;
	mins = 0;
	sprintf(CTtime, "--:--");
}

/**
 * RDS adatok feldolgozása
 */
void RDA5807M::ProcessRdsData() {

	//getStatusRegisters(); ->> getRadioInfo() metódusba kiemelve!

	//Ha nincs RDS szinkron, akkor törlünk mindent
	if (!reg0a->refined.RDSS) {
		clearRDS(); // reset all the RDS info.
		return;
	}

	rds_blockb blkb;
	blkb.blockB = reg0d->RDSB;

	uint8_t groupType = blkb.refined.groupType;

//#ifdef RDS_SERIAL_DEBUG
//	Serial.print("PTY: ");
//	Serial.println(blkb.refined.programType);
//#endif

	//http://www.g.laroche.free.fr/english/rds/groupes/listeGroupesRDS.htm
	//A switch/case nem stabil ennél a kódnál már, ezért van if/else if szerkezet :(
	if (groupType == 0x00 || groupType == 0x01) {
		// The data received is part of the Service Station Name
		int idx = 2 * (reg0d->RDSB & 0x0003);
		if (idx >= MAX_STATION_NAME_BUFF_LEN - 2) {
			idx = MAX_STATION_NAME_BUFF_LEN - 2;
		}

		// new data is 2 chars from block 4
		char c1 = reg0f->RDSD >> 8;
		char c2 = reg0f->RDSD & 0x00FF;

		// check that the data was received successfully twice
		// before sending the station name
		if ((PSName1[idx] == c1) && (PSName1[idx + 1] == c2)) {
			// retrieve the text a second time: store to _PSName2
			PSName2[idx] = c1;
			PSName2[idx + 1] = c2;
			PSName2[MAX_STATION_NAME_LEN] = '\0';

			if (strcmp(PSName1, PSName2) == 0) {

				// populate station name
				for (int n = 0, i = 0; i < MAX_STATION_NAME_LEN; i++) { // remove non-printable error ASCCi characters
					if (PSName2[i] > 31 && PSName2[i] < 127) {
						StationName[n] = PSName2[i];
						n++;
					}
				}
			}
		}

		if ((PSName1[idx] != c1) || (PSName1[idx + 1] != c2)) {
			PSName1[idx] = c1;
			PSName1[idx + 1] = c2;
			PSName1[MAX_STATION_NAME_LEN] = '\0';
		}

	} else if (groupType == 0x02) {
		int textAB = (reg0d->RDSB & 0x0010);
		int idx = 4 * (reg0d->RDSB & 0x000F);

		if (idx < lastTextIDX) {
			// The existing text might be complete because the index is starting at the beginning again.
			// Populate RDS text array.
			for (int n = 0, i = 0; i < strlen(RDSTxt); i++) {
				if (RDSTxt[i] > 31 && RDSTxt[i] < 127) {    // remove any non printable error charcters
					RDSText[n] = RDSTxt[i];
					n++;
				}
			}
		}
		lastTextIDX = idx;

		if (textAB != lasttextAB) {
			// when this bit is toggled the whole buffer should be cleared.
			lasttextAB = textAB;
			memset(RDSTxt, 0, sizeof(RDSTxt));
			memset(RDSText, '\0', sizeof(RDSText));
		}

		if (reg0b->refined.BLERA < 4) {

			// new data is 2 chars from block 3
			RDSTxt[idx] = (reg0e->RDSC >> 8);
			idx++;
			RDSTxt[idx] = (reg0e->RDSC & 0x00FF);
			idx++;

			// new data is 2 chars from block 4
			RDSTxt[idx] = (reg0f->RDSD >> 8);
			idx++;
			RDSTxt[idx] = (reg0f->RDSD & 0x00FF);
			idx++;
		}
	} else if (groupType == 0x04) {

#ifdef RDS_SERIAL_DEBUG
		Serial.println("RDS -> Clock time and date!");
#endif

		// Clock time and date
		if (reg0b->refined.BLERA < 3) { // allow limited RDS data errors as we have no correctioin code
			offset = (reg0f->RDSD) & 0x3F; // 6 bits
			mins = (reg0f->RDSD >> 6) & 0x3F; // 6 bits
			mins += 60 * (((reg0e->RDSC & 0x0001) << 4) | ((reg0f->RDSD >> 12) & 0x0F));
		}

		// adjust offset
		if (offset & 0x20) {
			mins -= 30 * (offset & 0x1F);
		} else {
			mins += 30 * (offset & 0x1F);
		}

		if (mins == lastmins + 1) { // get CT time twice before populating time
			minutes = mins;
		}
		lastmins = mins;

		if (reg0a->refined.RDSS) {
			if (minutes > 0 && minutes < 1500) {
				sprintf(CTtime, "%02d:%02d", (minutes / 60), (minutes % 60));
			}
		}
	}
}

//--- Infó ------------------------------------------------------------------------------

/**
 * Infók lekérése
 * Ez ametódus az összes státusz regisztert kiolvassa, így az RDS adatokat is
 */
void RDA5807M::getRadioInfo(RADIO_INFO *info) {

	//Írható olvasható regiszterek
	info->rdsEnabled = reg02.refined.RDS_EN;
	info->bassBoost = reg02.refined.BASS;
	info->softMute = reg04.refined.SOFTMUTE_EN;

	//Státusz (csak olvasható) regiszterek lekérése
	getStatusRegisters();

	info->rssi = reg0b->refined.RSSI;
	info->tuned = reg0a->refined.STC == 1;
	info->stereo = reg0a->refined.ST;
	info->rdsSync = reg0a->refined.RDSS; //RDS dekóder szinkronizált
	info->rdsAvail = (reg0a->refined.RDSS && reg0b->refined.ABCD_E == 0 && reg0b->refined.BLERB == 0); //Returns true when the RDS system has valid information
}
