/*
Hardware input/output routines for Will's Train Layout
Author: Chris Moller, chris.moller@evonet.com, Aug2015

v.3.0.18 02Mar2018
*/

#ifndef WillsIO_h
#define WillsIO_h

#include "Arduino.h"


//================================================================
//                      RFID routines - headers
//================================================================


// Port can be 1 (=Pin19=RFIDIn), 2(=Pin17=RFIDOut), 3 (=Pin15=RFIDIn2 for railcar detect)

class RFID   //handle all input from the RFID readers on the serial ports
{
public:
	RFID(byte port);
	void init();   //initialise the I/O
	String poll();   //poll an RFID, build the internal string, and if complete, return a byte
		// otherwise, return 0x00h

private:
	byte _port;
	byte rfidAvailable();   //retrurn the number of chars in the serial buffer
	byte rfidRead();    //the next char from the buffer
	byte charsRead[4];   // char count for each serial port  - 0xFF = not started
	String buildString[4];		// where the strings for each serial port are built
};



//================================================================
//                      Shift Register I/O routines - headers
//================================================================


class IO   //handle points and TOTIs
{
public:
	IO(bool dummy);
	void init(bool clearVars);   //initialise the I/O - if clearVars set, empty EEPROM
		//load point values from EEPROM into RAM
	void updater();	//ensure hardware and software agree

	bool testToti(byte totiNo);	//return whether a TOTI is occupied
	void setPoint(byte pointNo, bool set);   //set or clear a point
	bool getPoint(byte pointNo);  //return whether a point is set
	bool addToQueue(byte queue);  //push an exit onto the queue (return false if full)
	byte getFromQueue();   //fetch an exit from the queue 0x00 if nothing
	bool queueNotEmpty();  //test if there is anything in the queue
	void setExitModeDisplay(byte myExit);  //set flashing in the Exit Mode Display
	bool isThisSidingQueued(byte siding); //find out if this siding is already in the queue
	void clearActiveExit();  //forget what we've just be doing with EXIT

private:
	unsigned long pointValues;   //bit 0 is point 1, etc.  Off-normal if bit is set
	unsigned long totiValues;	//bit 0 is toti 1 etc.  Bit set if section occupied
  void setP1(byte pointNo, bool set);   //set or clear a point

	static bool blinker;
	int halfSecond = 25;
#define EXITQUEUELENGTH 4
	byte exitQueue[EXITQUEUELENGTH];  //lsn=siding#, msn=mode, lowest index goes next
	byte activeExit;  //last value popped from queue

	//EXIT destinations
const byte MAIN = 0x10;
const byte GOODS = 0x20;
const byte BRANCH = 0x40;

	byte exitModeDisplay = 0;  //MSN bit set as above (from queue), LSN also set if active
	//bits are THROUGH | BRANCH | GOODS | MAIN | THROUGH FLASH | BRANCH FLASH | GOODS FLASH | MAIN FLASH

	//in EEPROM:
	//byte EEpoint[32];
  //byte StoredTrain[8];
  // at 0x100...0x27F nvStates  



};



//================================================================
//                      User buttons - headers
//================================================================


class Buttons   //handle user buttons
{
public:
	Buttons();
	void init();   //initialise the counters
	String poll();	//check statuses
	/*returned value is a string containing the name of the significant button/function
	followed by a space and '1' to indicate that the button has just been pressed, 
	or '0' to indicate its release.  
	
	An exmpty string indicates nothing significant has happened.

	The strings are:
	  Up 
	  Down
	  Main
	  Goods
	  Branch
	  Through
	  Test
	  */

private:
	unsigned int buttonPins;    //same bit assignments as output
	unsigned int bounceCount;
	unsigned int previousPins;
	unsigned int lastAnnouncedValue;	  //last non-null output sent


};


//================================================================
//                      State machine operation - headers
//================================================================


class State   //handle State Machines
{
public:
	State(int machine);   //1=MERGE, 2=ENTER, 3=EXIT
	void init(bool clearVars);   //zero timers, fetch states from EEPROM.  If clearVars set, zero EEPROM
	void moveToState(byte newState);  //move to a new state value
	byte fetch();  //fetch the current state of this machine

private:
	int _machine;   //1=MERGE, 2=ENTER, 3=EXIT 
	byte myState[5];  //if msb clear, then we are just entering the state for the first time
};


//================================================================
//                      Timers - headers
//================================================================


class Timer   //handle timeouts
{
public:
	Timer(int myTime);
	void init(unsigned int seconds);   //start timer - set to zero to disable
	void tick();  //come here once a second, decrement timer, set expired flag TRUE if 1->0 
	bool expired();  //test expired flag

private:
	int _myTime;
	unsigned int counter[10];   //10 timers allowed
	bool expiredFlag[10];    //if we expired

};


//================================================================
//                      Beeper - headers
//================================================================


class Beeper   //make a noise
{
public:
	Beeper();
	void out(unsigned int milliseconds);   //sound 1kHz for specified duration

private:

};





//================================================================
//                      LCD routines - headers
//================================================================

//Tested - see Display.ino

class Display   //output all information messages to the LCD and perhaps the serial port
{
public:
	Display();
	void init(bool testMode);   //initialise the I/O
	//You MUST have Test Mode turned off during Visual Studio debugging, or breakpoints won't work!

	void out(String text);  //send the string to the LCD and perhaps the serial port too
	/*if a single line, then bottom line scrolls up to top line.
	Linefeeds are indicated by a vertical bar '|', since '\n' causes issues
	If string contains '|' then both lines are written.  Multiple '|' will only make sense to serial. */
	void tick();  //Come here every second to update the display if necessary
private:
	bool _testMode;  //remember internally whether we're going to output to serial
	String _bottomLine;  //what we last sent to the bottom line
	String fifo[4][32];  //text queued up to be displayed by tick()
//	bool _dummy;
};

#endif
