/*
 * RDA5807M.h
 *
 *  Created on: 2022. okt. 25.
 *      Author: BT
 */

#ifndef RDA5807M_H_
#define RDA5807M_H_

#include "defaults.h"
#include "RDA5807M_Registers.h"


#define RADIO_MAX_RSSI	0b111111 /* 63d */

typedef struct RADIO_INFO {
	int rssi;  				// Radio Station Strength Information.

	bool rdsEnabled;		// RDS engedélyezve?
	bool rdsSync;			// RDS szinkronizált
	bool rdsAvail; 			// RDS information is available.
	bool tuned;    			// A stable frequency is tuned.

	//--
	bool stereo;   			// Stereo audio is available
	bool bassBoost;			//bassBoost
	bool softMute;			//soft mute
};

/**
 *
 */
class RDA5807M {

private:
	const uint16_t startBand[4] = { 8700, 7600, 7600, 6500 };
	const uint16_t endBand[4] = { 10800, 9100, 10800, 7600 }; //!< End FM band limit
	const uint16_t fmSpace[4] = { 100, 200, 50, 25 };

private:

	uint16_t currentFrequency;
	uint8_t currentVolume;

	rda_reg02 reg02;
	rda_reg03 reg03;
	rda_reg04 reg04;
	rda_reg05 reg05;
	//rda_reg06	reg06; //nem kell
	rda_reg07 reg07;

	uint16_t shadowStatusRegisters[6]; //!< shadow status registers
	rda_reg0a *reg0a = (rda_reg0a*) &shadowStatusRegisters[0]; // SH_REG0A;
	rda_reg0b *reg0b = (rda_reg0b*) &shadowStatusRegisters[1]; // SH_REG0B;
	rda_reg0c *reg0c = (rda_reg0c*) &shadowStatusRegisters[2]; // SH_REG0C;
	rda_reg0d *reg0d = (rda_reg0d*) &shadowStatusRegisters[3]; // SH_REG0D;
	rda_reg0e *reg0e = (rda_reg0e*) &shadowStatusRegisters[4]; // SH_REG0E;
	rda_reg0f *reg0f = (rda_reg0f*) &shadowStatusRegisters[5]; // S

	void setRegister(uint8_t reg, uint16_t value);
	void* getStatusRegister(uint8_t reg);
	void getStatusRegisters();

	uint16_t getRealChannel();
	uint16_t getRealFrequency();
	uint16_t getRealFrequency(uint16_t realChannel);
	void waitAndFinishTune();

public:
	void init();
	void powerUp();
	void powerDown();

	void setBand(uint8_t band);
	void setSpace(uint8_t space);
	void setFmDeemphasis(uint8_t de);
	void setChannel(uint16_t channel);

	void setFrequency(uint16_t frequency);
	uint16_t getFrequency() {
		return this->currentFrequency;
	}

	void setFrequencyUp();
	void setFrequencyDown();

	void seek(uint8_t seek_mode, uint8_t direction);
	void seek(uint8_t seek_mode, uint8_t direction, void (*showFunc)());

	void setVolume(uint8_t value);
	uint8_t getVolume() {
		return this->currentVolume;
	}
	void setVolumeUp();
	void setVolumeDown();

	void setBass(bool value);
	void setSoftmute(bool value);
	void setMono(bool value);
	void setSeekThreshold(uint8_t value);

	void clearRDS();
	void setRDS(bool value);
	void setRdsFifo(bool value);
	void clearRdsFifo();
	void ProcessRdsData();
	void ProcessRdsData2();

	void getRadioInfo(RADIO_INFO *info);

private:
#define MAX_RDS_TXT_LEN	66
#define MAX_STATION_NAME_LEN 8
#define MAX_STATION_NAME_BUFF_LEN (MAX_STATION_NAME_LEN + 2)
#define MAX_TIME_BUFF_LEN 10

	// ----- actual RDS values
	int lasttextAB, lastTextIDX;

	int mins;           // RDS time in minutes
	int lastmins;
	char RDSTxt[MAX_RDS_TXT_LEN];
	char PSName[MAX_STATION_NAME_BUFF_LEN];    // including trailing '\00' character.
	char PSName1[MAX_STATION_NAME_BUFF_LEN];
	char PSName2[MAX_STATION_NAME_BUFF_LEN];

public:
	char StationName[MAX_STATION_NAME_BUFF_LEN];    // Station Name. 8 characters
	char RDSText[MAX_RDS_TXT_LEN];        // RDS test message 64 characters
	char CTtime[MAX_TIME_BUFF_LEN];         // CT time string formatted as 'hh:mm'
	int minutes;            // CT minutes transmitted on the minute
	int offset;         // RDS time offset and sign


};

#endif /* RDA5807M_H_ */
