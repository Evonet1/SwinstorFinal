/*
Hardware input/output routines for Will's Train Layout
Author: Chris Moller, chris.moller@evonet.com, Aug2015

v.3.0.18 02Mar2018
Changed if to while in line 66 - 07Nov2019
*/

#include "Arduino.h"
#include "EEPROM.h"
#include "WillsIO.h"
#include <LiquidCrystal.h>

#define BEEPPIN 6

//The Arduino seems to need this for stable operation
#define EEPROMupdateTime 7


//================================================================
//                      RFID routines - source
//================================================================


// Note that you should not use RFID chips with the lsb = 0x00,0xFF as these have special meanings
// Also, only use one RFID chip with each lsb value - this is the only byte that is significant


RFID::RFID(byte port)  //constructor
{
	_port = port;
}


void RFID::init()  //initialise the I/O
{
	switch (_port) {
	case 0:
		Serial.begin(9600);
		break;
	case 1:
		Serial1.begin(9600);
		break;
	case 2:
		Serial2.begin(9600);
		break;
	case 3:
		Serial3.begin(9600);
		break;
	default:  //take no action unless port is 0...3
		break;
	}
	//there will be four copies of these private variables:
	charsRead[_port] = 0xFF;
	buildString[_port] = "";
}


String RFID::poll() {

	//Check the serial port, and if there is a char, build the RFID string.  
	// If we get to 12 chars, take action
	// This routine could also be used for the USB serial port

	// This routine is called every 20mS

	byte val = 0;  //the character read
	
	//Find out if the UART has a character available
	while (rfidAvailable() > 0) {
		val = rfidRead();		//get the char


		if (charsRead[_port] == 0xFF) {   //we have not yet started
			if (val == 0x02) {                  // header starts 
				charsRead[_port] = 0;
				buildString[_port] = "";
			}		//any other char before start - throw away
		}
		else {    //we have already started
			buildString[_port] += char(val);
			charsRead[_port]++;

			if ((charsRead[_port] == 12) || (val < '0')) {
				if (charsRead[_port] < 12) {
					buildString[_port] += String("**Error");
				}

				charsRead[_port] = 0xFF;  //start over
				String result = buildString[_port];
				buildString[_port] = "";

				return (result);
			}
		}
	}  // nothing available

	return("");
}


byte RFID::rfidAvailable() {
	//Find out if the UART has a character available
	// whichRfid can be 1 (=Pin19=RFIDIn), 2(=Pin17=RFIDOut), 3 (=Pin15=RFIDIn2 for railcar detect)
	switch (_port) {
	case 0:
		return Serial.available();
	case 1:
		return Serial1.available();
	case 2:
		return Serial2.available();
	case 3:
		return Serial3.available();
	default:
		return(-1);
	}
}

byte RFID::rfidRead() {
	//Read a byte from the correct RFID
	switch (_port) {
	case 0:
		return Serial.read();
	case 1:
		return Serial1.read();
	case 2:
		return Serial2.read();
	case 3:
		return Serial3.read();
	default:
		return(-1);
	}
}



//================================================================
//                      Shift Register I/O routines - source
//================================================================


//tested - see IOtest.ino


//EXIT modes - these refer to sidingMode
#define THROUGH 0x80
#define MAIN 0x10
#define GOODS 0x20
#define BRANCH 0x40


IO::IO(bool dummy)  //constructor
{
	//	_dummy = dummy;
}

// RFC Pin assignments
const int STROBE = 3;
const int CLOCK = 2;
const int DATAOUT = 4;
const int DATAIN = 5;
const int shiftLength = 24;   //this is the number of outputs we have to set using DPR boards

//EEPROM memory map
const unsigned int EEpoint = 0x000;   //base address for EEpoint[32];  xx00...xx1F
//const unsigned int StoredTrain = 0x020;   //base address for StoredTrain[8];  xx20...xx27

void IO::init(bool clearVars)  //initialise the I/O
{
	//initilalise the hardware
	pinMode(STROBE, OUTPUT);
	pinMode(CLOCK, OUTPUT);
	pinMode(DATAOUT, OUTPUT);
	pinMode(DATAIN, INPUT);
	pinMode(8, OUTPUT);   //STOP17 = point 29
	pinMode(9, OUTPUT);   //Stop25
	pinMode(10, OUTPUT);  //Stop26
	pinMode(11, OUTPUT);  //Stop27
	pinMode(12, OUTPUT);  //Stop28
	pinMode(26, OUTPUT);  //pin 26 = MAIN LED indicator
	pinMode(27, OUTPUT);  //pin 27 = GOODS
	pinMode(28, OUTPUT);  //pin 28 = BRANCH
	pinMode(29, OUTPUT);  //pin 29 = THROUGH
   
	if (clearVars) {	//zero pointValues and set EEPROM to 0xFFh
		pointValues = 0UL;
		for (int x = 0; x < 40; x++){
			EEPROM.update(EEpoint + x, 0xFF);  //0xFF is what EEPROM contains if never written
			delay(EEPROMupdateTime);
		}
	}

	addToQueue(0);		//initialise the queue

	//copy EEPROM values into pointValues
	for (int y = 0; y < 32; y++){
		if (getPoint(y+1)){
			bitWrite(pointValues, y, 1);
		}
		else {
			bitWrite(pointValues, y, 0);
		}
	}


}


void IO::updater() {
	//Send point values from pointValues out to points
	// and get current TOTI values into totiValues
	const int pulseWidth = 25;  //set pulse widths
	//This routine will take approx 100 * pulseWidth microseconds to execute

  static bool blinker = false;
	//The following array is to fix the strange order in which DPR boards have their relays
	const byte mapDPR[] = { 6, 4, 2, 0, 7, 5, 3, 1, 14, 12, 10, 8, 15, 13, 11, 9, 22, 20, 18, 16, 23, 21, 19, 17 };

	//reset the DPR board shift registers
	digitalWrite(STROBE, HIGH);
	delayMicroseconds(pulseWidth);
	digitalWrite(STROBE, LOW);
	delayMicroseconds(pulseWidth);
	for (int shiftIndex = 0; shiftIndex < shiftLength; shiftIndex++) {  //for all the shift registers

		//clock out all the point values to the DPR boards 
		if (bitRead(pointValues,mapDPR[shiftIndex])) {
			digitalWrite(DATAOUT, HIGH);
		}
		else {
			digitalWrite(DATAOUT, LOW);
		}

		delayMicroseconds(pulseWidth);
		//load in all the Toti values
		// change the next line if the sense of TOTI o/p is wrong
		bitWrite(totiValues, shiftLength - 1 - shiftIndex, 1 - digitalRead(DATAIN));
		delayMicroseconds(pulseWidth);
		digitalWrite(CLOCK, HIGH);
		delayMicroseconds(pulseWidth);
		digitalWrite(CLOCK, LOW);
		delayMicroseconds(pulseWidth);
	}
	digitalWrite(STROBE, HIGH);
	delayMicroseconds(pulseWidth);
	digitalWrite(STROBE, LOW);
	delayMicroseconds(pulseWidth);

	//Now do the top five points/stop sections (25-29)
	//Assumes D9..D12, D8 are the Arduino pins for these
	for (int shiftIndex1 = shiftLength; shiftIndex1 < shiftLength + 5; shiftIndex1++) {
		int ioBit = shiftIndex1 - 15;
		if (ioBit == 13) {
			ioBit = 8;  //STOP17 = Point 29 is out of order on pin 8
		}
		if (bitRead(pointValues,shiftIndex1)) {
			digitalWrite(ioBit, HIGH);
		}
		else {
			digitalWrite(ioBit, LOW);
		}
	}

	if (--halfSecond == 0){
		//come here every half second
		halfSecond = 25;
		blinker = !blinker;

		//now update the Exit Mode display
		//bits are THROUGH | BRANCH | GOODS | MAIN | THROUGH FLASH | BRANCH FLASH | GOODS FLASH | MAIN FLASH
		//           pin29 | pin28  | pin27 | pin26
		bool pinVal;
		for (int modeIndex = 0; modeIndex < 4; modeIndex++) {  //for each destination
			if (bitRead(exitModeDisplay, modeIndex + 4)) {   //get the bit
				if (bitRead(exitModeDisplay, modeIndex)) {
					pinVal = blinker;
				} else { 
					pinVal = true;
				}
			}
			else {
				pinVal = false;
			}
			digitalWrite(26 + modeIndex, pinVal);
			//pin 26 = MAIN
			//pin 27 = GOODS
			//pin 28 = BRANCH
			//pin 29 = THROUGH
		}

	}

}


bool IO::testToti(byte totiNo) {
	//return whether a TOTI section is occupied
	byte totiNo1 = (totiNo - 1) & 0x1F;
	return(bitRead(totiValues, totiNo1));
}

void IO::setP1(byte pointNo, bool set) {
//set a point 1...32 to the value of SET
  
  byte pointNo1 = (pointNo -1) & 0x1F;
  bitWrite(pointValues, pointNo1, set);   //will take effect on next update()
  if (set){
    EEPROM.update(EEpoint + pointNo1, 0xFE);  //make a non-volatile copy
  }
  else {
    EEPROM.update(EEpoint + pointNo1, 0xFF);  //make a non-volatile copy
  }
  //This delay needed to be increased from 4, 2020-01-22, as Arduino was continually resetting
  delay(EEPROMupdateTime);
}

void IO::setPoint(byte pointNo, bool set){
  //set or unset the specified point
  //If point 0 is specified, clear all siding entry points
  if (pointNo == 0 ) {
    for (int spCount = 8; spCount >0; spCount--){
      setP1(spCount,false);
    }
  }
  else {
    setP1(pointNo,set);
  }
}

bool IO :: getPoint(byte pointNo) {   
//return whether a point is set or clear
	// - relies on lsb of value in EEPROM being correctly set
	// pointNo = 1...32
	byte pointVal = EEPROM.read(EEpoint + ((pointNo -1) & 0x1F));
	delay(1);
	if (bitRead(pointVal,0)) {
		return(false);
	}
	else {
		return(true);
	}
}


void IO::setExitModeDisplay(byte myExit){
	//Set the LEDs on the control panel from the queue
	//Set flashing the LED from myExit, which is the value just retrieved from the queue
	//Values for activeMode are:
	//Siding n to MAIN = 0x1n
	//Siding n to GOODS = 0x2n
	//Siding n to BRANCH = 0x4n
	//THROUGH to MAIN = 0x90
	//THROUGH to GOODS = 0xA0
	//THROUGH to BRANCH = 0xC0
	//Nothing yet active = 0x00

	// MSN of mode = destination, LSN = source
	// exit mode bits are mostly set from the queue, but also set from the activeMode parameter
	// if a bit is set from activeMode, it will also be set to flash
	// MSN uses ExitMode values (eg MAIN = 0X10)
	//if activeMode Siding (lsn) = 0, set THROUGH flashing as well

	exitModeDisplay = 0;
	//set exitModeDisplay MSN based on destinations stored in the queue
	for (int queueIndex = 0; queueIndex < EXITQUEUELENGTH; queueIndex++){
		exitModeDisplay = exitModeDisplay | (exitQueue[queueIndex] & 0xF0);  //set if there's something queued to this exit, strip siding
	}
	//Now determine which if any is flashing
	exitModeDisplay = exitModeDisplay | (myExit & 0xF0);  //Add currently active destination
	exitModeDisplay = exitModeDisplay | ((myExit & 0xF0) >> 4);  //Set currently active destination to flash

	halfSecond = 1;  //update soon
}


bool IO::addToQueue(byte queue) {
/*The queue consists of bytes with the MSN specifying the exit mode,
  the LSN the siding to despatch from.  exitQueue[0] is the next active entry */

//add an exit request onto the tail of the queue
	//If there's no space left, nothing will be registered, return(false)
	//to purge, write 0
	int writeQueue;
	if (queue == 0){  //purge queue
		for (writeQueue = 0; writeQueue < EXITQUEUELENGTH; writeQueue++){
			exitQueue[writeQueue] = 0;
		}
		activeExit = 0;
		setExitModeDisplay(0);	//stop any flashing indication
		return(true);
	} else {
		for (writeQueue = 0; writeQueue < EXITQUEUELENGTH; writeQueue++){
			if (exitQueue[writeQueue] == 0){  //empty queue position
				exitQueue[writeQueue] = queue;
				return(true);
			}
		}
	}
	return(false);
}

byte IO::getFromQueue(){
	//fetch the next exit request from the queue
	byte valToPop = exitQueue[0];
	//move all the others nearer the front
	for (int popIndex = 0; popIndex < EXITQUEUELENGTH - 1; popIndex++){
		exitQueue[popIndex] = exitQueue[popIndex + 1];
	}
	exitQueue[EXITQUEUELENGTH-1] = 0;
	activeExit = valToPop;  //only used by isThisSidingQueued
	return valToPop;
}

bool IO::isThisSidingQueued(byte siding){
	//find out if this siding is already in the queue or active
	for (int isqIndex = 0; isqIndex < EXITQUEUELENGTH; isqIndex++){
		byte isqVal = exitQueue[isqIndex];
		if ((isqVal != 0) && ((isqVal & 0x0F) == siding)) {
			return(true);  //this siding is queued
		}
		if ((activeExit != 0) && ((activeExit & 0x0F) == siding)) {
			return(true);  //this siding is active
		}

	}
	return false;
}

bool IO::queueNotEmpty() {
	//return true if there's something there
	return (exitQueue[0] != 0);
}

void IO::clearActiveExit(){
	//forget what we are doing in Exit state machine
	activeExit = 0;
}



//================================================================
//                      User buttons - source
//================================================================


//Digital control line assignments
const int buttonBase = 20;   //buttons must be in sequence
const int upButton = 20;       //D20 =1
const int downButton = 21;     //D21 =2
const int mainButton = 22;     //D22 =4
const int goodsButton = 23;    //D23 =8
const int branchButton = 24;   //D24 =16
const int throughButton = 25;  //D25 =32 


Buttons::Buttons()  //constructor
{

}




void Buttons::init()  //initialise the counters
{
	buttonPins = previousPins = bounceCount = lastAnnouncedValue = 0;
}

/////////////  This bit isn't done yet! ///////////////

String Buttons::poll() {
	// Act just once on each button-press, returning the strings indicated below to say what's happened

	struct key {    //See K&R p.124
		String keyWord;
		unsigned int keyVal;
	} keytab[] = {
		"Up 1", 0x0001,
		"Down 1", 0x0002,
		"Main 1", 0x0004,
		"Goods 1", 0x0008,
		"Branch 1", 0x0010,
		"Through 1", 0x0020,
		"Up 0", 0x0100,
		"Down 0", 0x0200,
		"Main 0", 0x0400,
		"Goods 0", 0x0800,
		"Branch 0", 0x1000,
		"Through 0", 0x2000,
		"Test 1", 0x0003,  //Up and Down together
		"Test 1", 0x0103,  //necessary because Up may get pressed first
		"Test 1", 0x0203,  // or Down
		"Cancel 1", 0x0028,  //Thru and Goods together
		"Cancel 1", 0x0228,   //needed in case Thru pressed first
		"Cancel 1", 0x0828,   //needed in case Goods pressed first 

		//you can add more key functions here if you need
		"X", 0xFFFF
	};

	buttonPins = (1 - digitalRead(upButton));  //Up = bit 0
	buttonPins += (1 - digitalRead(downButton)) * 2;   //Down = bit 1
	buttonPins += (1 - digitalRead(mainButton)) * 4;   //Main = bit 2
	buttonPins += (1 - digitalRead(goodsButton)) * 8;   //Goods = bit 3
	buttonPins += (1 - digitalRead(branchButton)) * 16;   //Branch = bit 4
	buttonPins += (1 - digitalRead(throughButton)) * 32;   //Through = bit 5

	if (buttonPins == previousPins) {
		bounceCount++;
	}
	else {
		bounceCount = 0;    //not stable
		previousPins = buttonPins;
	}
	if (bounceCount == 3){   //it's been this way for at least 80mS
		if (buttonPins != lastAnnouncedValue) {   //this is news
			unsigned int newAndOld = buttonPins + (lastAnnouncedValue *256);   //show previous state in msb
			lastAnnouncedValue = buttonPins;
			bounceCount = 0;
			for (int z = 0; keytab[z].keyWord != "X"; z++){
				if (keytab[z].keyVal == newAndOld) {
					return(keytab[z].keyWord);
				}
			}
//			return("Error");   //only do this if you need to flag illegal key combinations
			return("");

/*			String result = "";  //only do this to diagnose hardware issues
			for (int z = 0; z < 16; z++){
				result = (String)bitRead(newAndOld, z) + result;
			}
			return(result);  
*/		}
	}
	return("");

}


//================================================================
//                      State machines - source
//================================================================


State::State(int machine)  //constructor
{
	_machine = machine;
}

/* A copy of myState[] is held in EEPROM, using 3*256 bytes, to minimise
  wear on the EEPROM.  Only the LSB is significant (==0 for this state 
  being current).  The EEPROM is only read on power-up. 
  
  The MSB of myState is set if we are not entering this state for the first
  time (so only 127 states are possible for each State Machine).
  */

const int nvStates = 0x100;  //where we store the non-volatile states
//MERGE = 0x100...0x17f
//ENTER = 0x180...0x1ff
//EXIT  = 0x200...0x27f

void State::init(bool clearVars)  //initialise the state machines
{
	if (clearVars){
		for (int x = 0; x < 0x80; x++){
			EEPROM.update(nvStates + (0x080 * (_machine - 1)) + x, 0xFF);
			delay(EEPROMupdateTime);
		}
	}
	//fetch current state from nv memory, set RAM state to agree
	myState[_machine] = 0;   //initial state = 0
	int y;
	for (y = 0; y < 0x80; y++){
		if (bitRead(EEPROM.read(nvStates + (0x080 * (_machine - 1)) + y), 0) == false){
			delay(1);
			myState[_machine] = y;
			break;
		}
	}
}

void State::moveToState(byte newState) {

	//if newState is the same as the existing state, then set MSB
	byte oldState = myState[_machine];
	//remove existing nv state
	EEPROM.update(nvStates + (0x100 * (_machine - 1)) + oldState, 0xFF);
	delay(EEPROMupdateTime);
	if ((oldState & 0x7F) == newState) {
		myState[_machine] = (newState + 128);   //change the state in RAM, show not 1st time

	} else {
		myState[_machine] = newState ;   //change the state in RAM
	}
	//save it to nv memory too - but without the msb set - so always first time on power up
	EEPROM.update(nvStates + (0x080 * (_machine - 1)) + (myState[_machine] & 0x7F), 0xFE);
	delay(EEPROMupdateTime);
}


byte State::fetch() {  //fetch the current value of the state

	return myState[_machine];
}



//================================================================
//                      Timers - source
//================================================================


Timer::Timer(int myTime)  //constructor
{
	_myTime = myTime;
}

void Timer::init(unsigned int seconds)  //initialise the timer
{
	counter[_myTime] = seconds;
	expiredFlag[_myTime] = false;
}


void Timer::tick() {  //come here every second
	if (counter[_myTime] > 0) {    //if timer is running
		if (--counter[_myTime] == 0) {
			expiredFlag[_myTime] = true;   //only if 1->0
		}
	}
}

bool Timer::expired() {     //this will only indicated expiry ONCE per timer initialisation
	if (expiredFlag[_myTime] == true) {
		expiredFlag[_myTime] = false;
		return(true);
	}
	return(false);
}


//================================================================
//                      Beeper - source
//================================================================

Beeper::Beeper()  //constructor
{
}


void Beeper::out(unsigned int milliseconds)  
{
  //Note that this does not hold the program up!!
  //...but it does need interrupts enabled!
	tone(BEEPPIN, 1000, (unsigned long)milliseconds);    //1kHz
}



//================================================================
//                      LCD routines - source
//================================================================

// LCD code from http://www.arduino.cc/en/Tutorial/LiquidCrystal

// include the library code:
#include <LiquidCrystal.h>
// construct the lcd with the numbers of the interface pins
LiquidCrystal lcd(30, 32, 31, 37, 38, 39, 40);


Display::Display()  //constructor
{

}




void Display::init(bool testMode)  //initialise the display
{
	if (testMode) {
		Serial.begin(9600);
		delay(1000);  // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1289878242 
	}
	// set up the LCD's number of columns and rows: 
	lcd.begin(16, 2);    //set size of display
	_bottomLine = "          ";
	_testMode = testMode;
}


void Display::out(String text) {
	//print to the LCD display and perhaps the serial port

	/*Special characters are:
	'!' at the end of a line (causes a beep)
	'$' at the start of the line suppresses LF, but only if 
	    '$' also at the start of the current bottom line
	'|' in the text causes a CRLF.
	A string should not contain both '$' and '|'.
	*/

	int nlPos = -1;
	String temp = text;
	if (temp.endsWith("!")) {    //indicates an error
		tone(BEEPPIN, 1000, 250);    //500Hz, 300mS
	}

	bool noLF = ((temp.startsWith("$")) && (_bottomLine.startsWith("$")));

	while (temp.length() > 0) {  //while there is some text

		if (noLF == false) {   //unless we're going to overwrite
			//move bottom line to top
			lcd.setCursor(0, 0);
			lcd.print((_bottomLine + "                 ").substring(0, 16));
		}

		//now do bottom line
		nlPos = temp.indexOf ('|');
		if (nlPos > -1) {   // if there is a newline,
			if (nlPos > 0) {
				_bottomLine = temp.substring(0, nlPos);   //split it there
			}
			else {
				_bottomLine = "";   //if \n is first char
			}
			if ((int)temp.length() > nlPos) {
				temp = temp.substring(nlPos + 1);
			}
			else {
				temp = "";
			}
		}
		else {
			_bottomLine = temp;  //otherwise, display the whole string
			temp = "";
		}
		lcd.setCursor(0, 1);
		lcd.print((_bottomLine + "                 ").substring(0, 16));

		if (_testMode) {
			Serial.println(_bottomLine);
			delay(1000);   //give time for buffer to empty so as not to upset debug
			//but note that this will make all timings wrong, so strictly for debugging!
		}
	}
}



void Display::tick()  //update the display every second if necessary
{

}
