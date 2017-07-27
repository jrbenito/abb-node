// ABB Inverter Node
//
// This node communicates with ABB Inverters through RS485 and 
// sends data to a central node/gateway using a RFM69 radio.
/////////////////////////////////////////////////////////////////////////////////////
// Copyright Josenivaldo Benito Junior 2017
// http://benito.com.br/
/////////////////////////////////////////////////////////////////////////////////////
// License
/////////////////////////////////////////////////////////////////////////////////////
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
/////////////////////////////////////////////////////////////////////////////////////
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlashA.h>      //get it here: https://github.com/lowpowerlab/spiflash
#define SPIFlash SPIFlashA
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <Thread.h>        // execution threads
#include <ThreadController.h>

//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID        2     // node ID used for this unit
#define NETWORKID   100
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY_EXACT 916000000
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*****************************************************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75
//*****************************************************************************************************************************
//#define BR_300KBPS             //run radio at max rate of 300kbps!
//*****************************************************************************************************************************
#define ACK_TIME    30  // # of ms to wait for an ack
#define ENCRYPTKEY "sampleEncryptKey" //(16 bytes of your choice - keep the same on all encrypted nodes)
//*****************************************************************************************************************************
#define BLINKLED_PERIOD 1000
#define BATTERY_STATUS_PERIOD 5000

// Onboard LED and flash chip select pins
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Miniwirless have LEDs on D9
  #define FLASH_SS      5 // and FLASH SS on D8
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(FLASH_SS, 0x12018); //12018 for 128Mbit Spanion flash

ThreadController controll = ThreadController();
int batteryVcc = 0;
Thread batteryStatus = Thread();
Thread blinkLed = Thread();

void batteryStatusWorker() {   // return vcc voltage in millivolts
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);             // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate in mV

  batteryVcc = (int)result;

  return;
}

void blinkLedWorker() {
    static bool ledStatus = false;
	ledStatus = !ledStatus;

	digitalWrite(LED, ledStatus);
}

void setup() {
    pinMode(LED, OUTPUT);

	// Radio setup
	radio.initialize(FREQUENCY, NODEID, NETWORKID);
	radio.encrypt(ENCRYPTKEY);
#ifdef FREQUENCY_EXACT
	radio.setFrequency(FREQUENCY_EXACT);
#endif
#ifdef ENABLE_ATC
	radio.enableAutoPower(ATC_RSSI);
#endif
#ifdef IS_RFM69HW_HCW
	radio.setHighPower();
#endif
#ifdef BR_300KBPS
	radio.writeReg(0x03, 0x00);  //REG_BITRATEMSB: 300kbps (0x006B, see DS p20)
	radio.writeReg(0x04, 0x6B);  //REG_BITRATELSB: 300kbps (0x006B, see DS p20)
	radio.writeReg(0x19, 0x40);  //REG_RXBW: 500kHz
	radio.writeReg(0x1A, 0x80);  //REG_AFCBW: 500kHz
	radio.writeReg(0x05, 0x13);  //REG_FDEVMSB: 300khz (0x1333)
	radio.writeReg(0x06, 0x33);  //REG_FDEVLSB: 300khz (0x1333)
	radio.writeReg(0x29, 240);   //set REG_RSSITHRESH to -120dBm
#endif

	flash.initialize();

	// Thread initialization
	batteryStatus.onRun(batteryStatusWorker);
	batteryStatus.setInterval(BATTERY_STATUS_PERIOD);

	blinkLed.onRun(blinkLedWorker);
	blinkLed.setInterval(BLINKLED_PERIOD);

	controll.add(&batteryStatus);
	controll.add(&blinkLed);
}

void loop() {
    // run threads
    controll.run();

	// Check for existing RF data, potentially for a new sketch wireless upload
	// For this to work this check has to be done often enough to be
	// picked up when a GATEWAY is trying hard to reach this node for a new sketch wireless upload
	if (radio.receiveDone())
	{
		Serial.print("Got [");
		Serial.print(radio.SENDERID);
		Serial.print(':');
		Serial.print(radio.DATALEN);
		Serial.print("] > ");
		for (byte i = 0; i < radio.DATALEN; i++)
			Serial.print((char)radio.DATA[i], HEX);
		Serial.println();
		CheckForWirelessHEX(radio, flash, false, 9);
		Serial.println();
	}
}
