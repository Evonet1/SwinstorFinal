//================================================================
//			 Arduino software for Swindon Station Storage Yards
//================================================================
// Requires Arduino Mega2560 - on account of needing 3 UARTs

// The layout is controlled by two Arduino Mega2560s - EAST & WEST
// EAST controls traffic heading towards WEST (down) and storage for traffic from WEST (up)
// WEST controls traffic heading towards EAST (up) and storage for traffic from EAST (down)

//This version needs TOTI(DCCCHECKTOTI) to be permanently occupied as long as DCC is on

//Put in jumper link J1 on the shield used for West
//Ground D41 if you want to protect EEPROM from being written with train details

const String swVersion = "3.2.16 01Mar2020";


const bool DEBUG = false;
//Set this true to output everything that goes to the display to the serial port as well
//Note that in this case, the program will NOT keep time! 

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "WillsIO.h"


int testMode = 0;
int rfidCount = 1;

//State machines
const int UNOWNED = 0;	//No state machine owns this area
const int MERGE = 1;
const int ENTER = 2;
const int EXIT = 3;
const int INTERLOPER = 4;	//an interloper has appeared in the protected area
const int PROTAREATOTI = 12;   //changed from 20
const int SCISSORSAREATOTI = 14;

const int DCCCHECKTOTI = 24;	//TOTI always occupied to tell whether DCC is on
const bool DCCCHECKDISABLED = false;	 //true if TOTI24 is not wired 

//assign classes
IO io(false);			 //input/output space
Display display;		//LCD display
Buttons buttons;		//user input
Beeper beeper;
State smMerge(MERGE);
State smEnter(ENTER);
State smExit(EXIT);
//We can support up to ten timers
Timer timer1(1);
Timer timer2(2);
Timer timer3(3);

RFID rfid1(1);	//ENTER RFID
RFID rfid2(2);	//EXIT RFID
RFID rfid3(3);	//Railcar RFID (not used)
//EEPROM array for remembering trains
const unsigned int StoredTrain = 0x020;   //base address for StoredTrain[8] in EEPROM;  xx20...xx27



const int ledPin = 13;	 // heartbeat
const int eastPin = 7;	 //Set East or West
const int writeEnable =	41;	//enable writing to EEPROM

bool lastWriteEnable;
bool timerFlag;	 //this flag is set by the interrupt routine every 20mS
int oneSecondCount = 50;
int STAYINSTATE = 20;	 //seconds allowed in state before deemed as stuck
bool overRun;		//this flag is set if loop() takes more than 20mS to exectute
const unsigned int timer1_counter = 64286;	 // preload timer 65536-16MHz/256/50Hz

int testIOAddress = 1;
bool firstFlag = true;
unsigned long newTotiValues, oldTotiValues;

int protArea = UNOWNED;		//The protected area of points 19,20,23 may be owned by MERGE or EXIT
int scissorsArea = UNOWNED;		//The crossover between sidings 1,2 may be owned by ENTER or EXIT
int lastPA = UNOWNED;		//previous ownership of protArea
int lastXA = UNOWNED;		//previous ownership of Xover Area

bool despatchMode;	 //if selecting a train to exit
bool throughMode;	//true if we want an ENTER train to go through
bool dccOn;				//true if DCC is on
bool lastDccCheck;	//Previous DCC on/off value
byte exitTrainId = 0xFF;   //RFID value of train exiting

byte myEnterSiding;
int exitSiding = -1;	 //current candidate siding for exit queue 0...8
//this is the value used in the user Destination selection interface


byte myExit;	 //what we are actually doing in the Exit state machine
//EXIT destinations
//Siding n to MAIN = 0x1n
//Siding n to GOODS = 0x2n
//Siding n to BRANCH = 0x4n
//THROUGH to MAIN = 0x90
//THROUGH to GOODS = 0xA0
//THROUGH to BRANCH = 0xC0

byte myExitSiding;	//the siding we're going to get the train from
byte myExitSiding1; //fixed Siding0 (through) = 15
byte myDestination;	//where it's going (use EXIT modes coding)
byte preferredSiding = 0xFF; //where an RFID says this train should go
byte thisTrainRfid;	//the RFID of the train seen entering the sidings


//EXIT destinations
const byte MAIN = 0x10;
const byte GOODS = 0x20;
const byte BRANCH = 0x40;
const byte THROUGH = 0x80;	//this isn't strictly a destination
//These values are added to the siding number, to store an exit request in the queue


void setup()
{

	display.init(DEBUG);
	display.out("Spirit ofSwindon");
	delay(1000);

//Explicitly initialise variables
  testMode = 0;
  rfidCount = 1;
  oneSecondCount = 50;
  STAYINSTATE = 20;   //seconds allowed in state before deemed as stuck
  testIOAddress = 1;
  firstFlag = true;
  protArea = UNOWNED;    //The protected area of points 19,20,23 may be owned by MERGE or EXIT
  scissorsArea = UNOWNED;    //The crossover between sidings 1,2 may be owned by ENTER or EXIT
  lastPA = UNOWNED;   //previous ownership of protArea
  lastXA = UNOWNED;   //previous ownership of Xover Area
  exitSiding = -1;   //current candidate siding for exit queue 0...8

	/*
	Hardware Timer1 Interrupt
	Flash LED every second
	Code from: http://www.hobbytronics.co.uk/arduino-timer-interrupts
	*/

	pinMode(ledPin, OUTPUT);
	pinMode(eastPin, INPUT);
	pinMode(writeEnable, INPUT);

	// initialize timer1 
	noInterrupts();					 // disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;

	// Set timer1_counter to the correct value for our interrupt interval
	//timer1_counter = 64886;	 // preload timer 65536-16MHz/256/100Hz
	//const unsigned int timer1_counter = 64286;	 // preload timer 65536-16MHz/256/50Hz
	//timer1_counter = 34286;	 // preload timer 65536-16MHz/256/2Hz

	TCNT1 = timer1_counter;	 // preload timer
	TCCR1B |= (1 << CS12);		// 256 prescaler 
	TIMSK1 |= (1 << TOIE1);	 // enable timer overflow interrupt
	timerFlag = false;
	overRun = false;

	if (DEBUG) {	//time out faster if debugging
		STAYINSTATE = 10;
	}
	else {
		STAYINSTATE = 30;
	}

	lastWriteEnable = digitalRead(writeEnable);

	timer1.init(STAYINSTATE);	 //set timer1 to 10secs
	timer2.init(0);		//disable timer2
	timer3.init(0);		//disable timer2
	oneSecondCount = 50;
	
	interrupts();						 // enable all interrupts

	rfid1.init();	 //initialise RFIDs
	rfid2.init();
	rfid3.init();


	//initialise points
	io.init(false);
	buttons.init();
	//initialise state machines
	smMerge.init(false);
	smEnter.init(false);
	smExit.init(false);

	display.out(swVersion);	//this shows that initialisation is complete
	delay(1000);
	if (digitalRead(eastPin)) {
		display.out("East Box");
	}
	else {
		display.out("West Box");
	}
	beeper.out(1000);
	delay(2000);	//wait for beeper to finish, and another second


	io.addToQueue(0);		//purge the exit queue 
	exitSiding = -1;

	dccOn = true;
	lastDccCheck = true;

	enterRunMode();

}

ISR(TIMER1_OVF_vect)				// interrupt service routine 
{
	TCNT1 = timer1_counter;	 // preload timer
	if (timerFlag == true) {	// loop has taken more than 20mS to execute
		overRun = true;
	}
	timerFlag = true;
}

void(* resetFunc)(void) = 0;  //declare reset function at address 0
//from https://www.instructables.com/id/two-ways-to-reset-arduino-in-software/


void loop()
{
	// come here every 20mS
	io.updater();

	dccOn = io.testToti(DCCCHECKTOTI);	 //determine whether DCC is on

	overRun = false;

	if (timerFlag){
		timerFlag = false;
		if (--oneSecondCount == 0){
			//come here every second
			oneSecondCount = 50;	//50Hz
			digitalWrite(ledPin, digitalRead(ledPin) ^ 1);	 //flash the pulse led
			//decrement second timers
			if ((dccOn) || DCCCHECKDISABLED) {	// suspend timers if DCC is off
				timer1.tick();
				timer2.tick();
				timer3.tick();
			}
		}


		//================================================================
		//							Here is where all the logic of the program goes
		//================================================================
		//We will come here every 20mS

		String myButtons = buttons.poll();

		if (myButtons == "Cancel 1") {
			byte xExit = io.getFromQueue();	 //remove one from top of queue, whichever mode we're in
			protArea = UNOWNED;	 //release Protected Area
			scissorsArea = UNOWNED;	 // release ownership of Siding1/2 crossover

			io.setExitModeDisplay(myExit);	//Exit still active
			String report = "";
			if (xExit == 0){
				display.out("Queue empty!");
			}
			else {
				if ((xExit & MAIN) != 0) report += "M";
				if ((xExit & GOODS) != 0) report += "G";
				if ((xExit & BRANCH) != 0) report += "B";
				report += (String)(xExit & 0x0F);
				display.out("Cancel exit " + report);
				smExit.moveToState(0);
			}
			enterRunMode();
		}


		if (testMode == 0){	 //Run mode or Despatch mode

			//this is where all the main logic goes

			///////////////////////////////////////////////////////////////

			if (dccOn || DCCCHECKDISABLED) {	 //only update states if DCC is on (otherwise TOTIs will all show CLEAR)

				if (!lastDccCheck) {
					display.out("DCC restored");
					beeper.out(500);
          delay(500);  //give time for TOTIs to register
					lastDccCheck = true;
				}

				if (DEBUG){
					//Show change of ownership of contested areas
					if ((scissorsArea != lastXA) || (protArea != lastPA)) {
						display.out("PA=" + (String)protArea + ", XA=" + (String)scissorsArea);
						lastXA = scissorsArea;
						lastPA = protArea;
					}
				}

				updateMerge(); // Merge State Machine
				//The Merge State Machine owns the following points:	20,21.
				//It -may- have the right to control points 19,23 - if it owns the protected area (protArea == MERGE)
				//It controls Stop Relays 25,26,27.
				//It considers the TOTI values of 11,20,21,22,23.

				updateEnter(); // Enter State Machine
				//The Enter State Machine owns the following points:	1,3,4,5,6,7,8.
				//It -may- have the right to control point 2 - if it owns the crossover (scissorsArea == ENTER)
				//It controls Stop Relay 28.
				//It considers the TOTI values of 1,2,3,4,5,6,7,8,9,11A.

				updateExit(); // Exit State Machine
				// The user interface buttons only have any effect in Exit, 
				// to select which train should exit the storage yards.

				//The Exit State Machine owns the following points:	9,10,11,12,13,14,15,16,17,18,19.
				//It -may- have the right to control point 2 - if it owns the crossover (scissorsArea == ENTER)
				//It controls Stop Relay 28.
				//It considers the TOTI values of 1,2,3,4,5,6,7,8,9,11A.

				//Check if we've seen an RFID
				String exitRfid;
				if ((exitRfid = rfid2.poll()) != "") {
					//If we see an Exit RFID, save the lsb of the train ID in EEPROM for the siding we've just exited
					exitTrainId = hexStrToByte(exitRfid);
					if ((myExitSiding > 0) && (myExitSiding < 9)) {
						//write the last two characters of the string
						if (digitalRead(writeEnable)) {   //this has become the Write-protect switch
							EEPROM.write(StoredTrain + myExitSiding - 1, exitTrainId);
						}
					}
					if (!despatchMode) {
					  reportStates(!digitalRead(writeEnable));
					}
				}
				String enterRfid;
				if ((enterRfid = rfid1.poll()) != "") {
					//If we see an Enter RFID, look up the EEPROM array to see if we know a siding for it
					byte searchForTrain;
					thisTrainRfid = hexStrToByte(enterRfid);
					preferredSiding = 0xfe;  //If we don't recognise the train, =0xfe (0xff is used for nothing heard)
					for (searchForTrain = 0; searchForTrain < 8; searchForTrain++) {
						if (EEPROM.read(StoredTrain + searchForTrain) == thisTrainRfid){
							preferredSiding = searchForTrain + 1;
						}
					}
				}

			}
			else {
				if (!dccOn	&& !DCCCHECKDISABLED && lastDccCheck) {	//DCC just failed
					display.out("No DCC!");
					lastDccCheck = false;
				}
			}

			if (myButtons != ""){

				if (myButtons == "Test 1"){
					testMode = 3;
					display.init(DEBUG);  //just in case display has got screwed
					display.out("Test mode");
					io.addToQueue(0);		//purge the exit queue when entering Test
					exitSiding = -1;
					beeper.out(500);
				}

				//Despatch mode
				if (despatchMode) {	 //we are already in despatch mode

					if (myButtons == "Through 0") {
						//Cancel Despatch mode
						io.setExitModeDisplay(myExit);	//just update from queue
						enterRunMode();

					}

					if (exitSiding != -1) { //there is something to queue
						if (myButtons == "Down 0") {

							exitSiding = findNextSiding(exitSiding, false);
						}
						if (myButtons == "Up 0") {
							exitSiding = findNextSiding(exitSiding, true);
						}
						if (myButtons == "Main 0"){
							byte myMain = (MAIN | exitSiding);
							if (exitSiding == 0) {
								myMain = myMain | THROUGH;
							}
							if (io.addToQueue(myMain)) {
								io.setExitModeDisplay(myExit);	//just update from queue
								display.out(sidingString(exitSiding) + "->Main");
							}
							else {
								display.out("Queue full!");
							}
							enterRunMode();
						}
						if (myButtons == "Goods 0"){
							byte myGoods = (GOODS | exitSiding);
							if (exitSiding == 0) {
								myGoods = myGoods | THROUGH;
							}
							if (io.addToQueue(myGoods)) {
								io.setExitModeDisplay(myExit);	//just update from queue
								display.out(sidingString(exitSiding) + "->Goods");
							}
							else {
								display.out("Queue full!");
							}
							enterRunMode();
						}
						if ((myButtons == "Branch 0") && !digitalRead(eastPin)){
							byte myBranch = (BRANCH | exitSiding);
							if (exitSiding == 0) {
								myBranch = myBranch | THROUGH;
							}
							if (io.addToQueue(myBranch)) {
								io.setExitModeDisplay(myExit);	//just update from queue
								display.out(sidingString(exitSiding) + "->Branch");
							}
							else {
								display.out("Queue full!");
							}
							enterRunMode();
						}

					}	//else ignore button, because there is no train to queue
				}

				//Run mode
				else { //we are in Run mode, not Despatch

					if (myButtons == "Through 0") {	 //select despatch mode
						despatchMode = true;
						display.out("Select exit");
						exitSiding = 8;  //to roll round to zero
						exitSiding = findNextSiding(exitSiding, true);
						if (exitSiding == -1){
							display.out("No train!");
						}
					}

					if (myButtons == "Cancel 1") {  //abandon all despatches
						display.out("Cancel exit");
						exitSiding = -1;
						smExit.init(true);
					}


					// do other run mode button actions here...

				}
			}
		}	 //end Run or Despatch

		else {			// we are in a test mode

			//whichever test mode we're in, simultaneous up and down will get us out
			if (myButtons == "Test 1"){
				enterRunMode();
			}
			else {

				String rfidString;	//for some reason, these can't be declared within the case
				String testString;

				switch (testMode) {

					/*			case 1:		// we are waiting to select a test mode
								  if (myButtons == "Main 1"){
								  testMode = 3;
								  display.out("Test I/O");
								  }
								  break;
								  */

				case 3:		//test I/O

					//Do user-driven points
					if (myButtons == "Up 1") {
						if (testIOAddress++ == 35) testIOAddress = 1;
					}
					if (myButtons == "Down 1") {
						if (testIOAddress-- == 1) testIOAddress = 35;
					}

					if (myButtons == "Through 1") {	 // toggle the point
						if (testIOAddress <= 29) {
							if (io.getPoint(testIOAddress)) {
								io.setPoint(testIOAddress, false);
								display.out("Point " + (String)(testIOAddress)+" clear");
							}
							else {
								io.setPoint(testIOAddress, true);
								display.out("Point " + (String)(testIOAddress)+" set");
							}
						}
						else {
							if (testIOAddress >= 32) {	//32..35	translate to D26..D29
								digitalWrite(testIOAddress - 6, !digitalRead(testIOAddress - 6));
							}
						}
					}

					if ((myButtons == "Up 1") || (myButtons == "Down 1") || (myButtons == "Through 1")) {
						testString = (String)(testIOAddress);
						if (testString.length() == 1) {	 //format = ##
							testString = "0" + testString;
						}
						testString = "Point/Toti" + testString;
						if (testIOAddress <= 29) {	//29 is the last point in use
							if (io.getPoint(testIOAddress)) {
								testString += "P1";
							}
							else {
								testString += "P0";
							}
						}
						else if (testIOAddress < 32) {
							testString += "--";
						}
						else {	//LEDs
							if (digitalRead(testIOAddress - 6)) {
								testString += "L1";
							}
							else {
								testString += "L0";
							}
						}
						// now add TOTI state
						if (testIOAddress <= 24){	//last TOTI in use is 24

							if (io.testToti(testIOAddress)) {
								testString += "T1";
							}
							else {
								testString += "T0";
							}
						}
						else {
							testString += "--";
						}

						display.out(testString);
					}


					if (myButtons == "Goods 1"){
						testMode = 5;
						display.out("Clear States");
					}

					//Report any changes to TOTIs
					for (byte totiCount = 0; totiCount < 32; totiCount++) {			//for each TOTI
						bool thisToti = io.testToti(totiCount + 1);
						bitWrite(newTotiValues, totiCount, thisToti);		//build new TOTI value
						if (firstFlag) {		//if this is the first time, make old the same
							bitWrite(oldTotiValues, totiCount, thisToti);
						}
						if (thisToti && !bitRead(oldTotiValues, totiCount)) {
							bitWrite(oldTotiValues, totiCount, thisToti);
							display.out("TOTI[" + (String)(totiCount + 1) + "] in use");
							beeper.out(500);
						}
						if (!thisToti && bitRead(oldTotiValues, totiCount)) {
							bitWrite(oldTotiValues, totiCount, thisToti);
							display.out("TOTI[" + (String)(totiCount + 1) + "] clear");
							beeper.out(500);
						}
					}
					firstFlag = false;

					if (lastWriteEnable != digitalRead(writeEnable)) {
						if (digitalRead(writeEnable)) {
							display.out("Read/Write");
						}
						else {
							display.out("Write-Protect");
						}
						lastWriteEnable = digitalRead(writeEnable);
					}

					//Report any inputs on RFID readers
					switch (rfidCount) {
					case 1:
						rfidString = rfid1.poll();
						break;
					case 2:
						rfidString = rfid2.poll();
						break;
					case 3:
						rfidString = rfid3.poll();
						break;
					default:
						break;
					}

					if (rfidString != ""){
						display.out("[" + (String)(rfidCount)+"]=" + rfidString);
						beeper.out(500);
					}
					if (rfidCount++ == 3) rfidCount = 1;

					break;

				case 5:	 //clear state machines, but do not forget train/siding assignments
					beeper.out(500);
					smMerge.init(true);
					smEnter.init(true);
					smExit.init(true);
					protArea = UNOWNED;  //release Protected Area
					scissorsArea = UNOWNED;   // release ownership of Siding1/2 crossover
//			          io.addToQueue(0);		//already done on entry to Test Mode
					display.init(DEBUG);
					display.out("States cleared");
					delay(1000);
					beeper.out(500);
					testMode = 3;
					break;

				default:
					beeper.out(2);
					break;
				}

			}
			//================================================================
			//							End the logic of the program
			//================================================================

		}

		if (overRun == true){
			overRun = false;
			//		beeper.out(2);	 //click if we've taken too long and missed a 20mS tick
		}

	}

}		//end of the loop



//================================================================
//							Update the MERGE State Machine
//================================================================

// The MERGE State Machine is controlled by TOTIs 11, 18-22.
// It drives points 20,21,22,23 and controls the entry of trains into the MERGE
// section using Stop Sections 25,26,27
// If merging from Branch, it takes ownership of the Protected Area, and in doing this, it may
//	 cause the EXIT State Machine to have to wait to despatch a train.


void updateMerge() {

	String interloper;

	byte state = smMerge.fetch();

	bool entryFlag = false;
	byte stripState = state & 0x7F;

	if (state <128) { //first time in this state
		entryFlag = true;
		smMerge.moveToState(state);	 //set the msb
		if (despatchMode == false) {   //suppress reporting if we're despatching
      reportStates(false);
		}
	}

	switch (stripState) {

	case 0 :	//IDLE

		if (entryFlag) {	//entering state for the first time
			io.setPoint(21, false);	 // clear point
			io.setPoint(23, false);	 // clear point
			io.setPoint(25, false);	 // Stop Goods
			io.setPoint(26, false);	 // Stop Main
			io.setPoint(27, false);	 // Stop Branch
			if (protArea == MERGE) {
				protArea = UNOWNED;
			}
		}

		if (io.testToti(11) || (io.testToti(12) && (protArea != EXIT)) || io.testToti(20)) {	//something has appeared unexpectedly
			interloper = "T";
			if (io.testToti(11)) {
				interloper += "11";
			}
			if (io.testToti(12)) {
				interloper += "12";
			}
			if (io.testToti(20)) {
				interloper += "20";
			}
			display.out(interloper + " Interloper!");
			smMerge.moveToState(10);    //Exception

		} else {

			if (io.testToti(21))  {
				//A train has appeared on the Main arriving TOTI
				smMerge.moveToState(1);		 //Main waiting
				break;
			}
			if (io.testToti(22))  {
				//A train has appeared on the Goods arriving TOTI
				smMerge.moveToState(11);		 //Goods waiting 
				break;
			}
			if ((io.testToti(23)) && (protArea == UNOWNED)) {
				//A train has appeared on the Branch arriving TOTI
				smMerge.moveToState(21);		 //Branch waiting 
				break;
			}
		}
		break;

    //****************** EXCEPTION STATE *************************
    
  case 10:  //Something has appeared unexpectededly

    if (!io.testToti(11) && !io.testToti(12) && !io.testToti(20)){ //the problem has gone
		smMerge.moveToState(0);
    }
    break;

		//****************** MAIN STATES *************************

	case 1:	 //Main waiting
		if (entryFlag){
			timer1.init(STAYINSTATE);
		}

		if (!io.testToti(20) && !io.testToti(11)) {	 
			//The way is clear
//		io.setPoint(20, false);	// No crossover (doesn't exist any more_
			io.setPoint(21, false);	// Set Goods/Main point to Main
			io.setPoint(23, false);	// Set Branch/Main to Main
			io.setPoint(25, false);	 // Stop Goods (Stop25)
			io.setPoint(27, false);	 // Stop Branch (Stop27)
			io.setPoint(26, true);	 // Allow Main to go (Stop26)
			smMerge.moveToState(2);		//Main can go
			break;
		}

		if (io.testToti(11) || io.testToti(20)){	//Leave Interloper to be handled by main loop
			smMerge.moveToState(0);
			break;
		}


	case 2:	//We have committed to allow Main to go, TOTI21 moving into TOTI20 (TOTI11 if EAST)
		//Give MAIN 20s to move - otherwise if someone else is waiting, it loses its turn
	 
		if (entryFlag){
			timer1.init(STAYINSTATE);
		}
	 
		if (io.testToti(20) == true) {	
			//watch for train moving into T20
			smMerge.moveToState(4);	//Main going
			break;
		}

		if (io.testToti(11)){ //Leave Interloper to be handled by main loop
			smMerge.moveToState(0);
			break;
		}
    
		if (io.testToti(21) == false) {	//Main disappeared without moving into T11/T20
			display.out("Main vanished!");
			io.setPoint(26, false);	 // Clear the signal (Stop26)
			smMerge.moveToState(0);
			break;
		}

		if (timer1.expired() == true) {
			display.out("Main stuck T21!");
			if (io.testToti(22) == true){	//Goods is waiting
				io.setPoint(26, false);	//Main can no longer go
				smMerge.moveToState(11);	//go to Goods waiting
				break;
			}
			if (io.testToti(23) == true){	//Branch is waiting
				io.setPoint(26, false);	//Main can no longer go
				smMerge.moveToState(21);	//go to Branch waiting
				break;
			}
     
			timer1.init(STAYINSTATE); //no-one else waiting, so go round again
		}

		break;

	case 4:	//Train moving, now front is in TOTI20 - only in WEST
		if (entryFlag){	
			timer1.init(STAYINSTATE);
		}
		if (io.testToti(11) == true){
			smMerge.moveToState(5);	//Merge from protected
			break;
		}
		if (io.testToti(20) == false){	
			//if T20 clear without going into T11, we must be backing out...
			smMerge.moveToState(0);
			break;
		}
		if (timer1.expired() == true) {
			display.out("Merge stuck T20!");
			timer1.init(STAYINSTATE);
		}
		break;

	case 5:	//Merge from protected, now front is in TOTI11
		//May be held here by other end's ENTER State Machine at STOP28
		// Stay here until TOTI11 are clear
		//Exceptional cases (eg TOTI20 not clear) will be handled in main loop
		if (entryFlag){
			io.setPoint(26, false);  // Clear the signal (Stop26)
		}

		if (!io.testToti(20)) { //no need to wait for T11 to be clear to allow exiting trains
			if (protArea == MERGE) {
				protArea = UNOWNED;
			}
		}
   
		if (io.testToti(11) == false)	{
			smMerge.moveToState(0);	//All done
		}
		break;


		//****************** GOODS STATES *************************

	
	case 11:	 //Goods waiting
		if (entryFlag){
			timer1.init(STAYINSTATE);
		}

		if (!io.testToti(20) && !io.testToti(11)) {    //The way is clear
			io.setPoint(21, true);	// Set Goods/Main point to Goods
			io.setPoint(23, false);	// Set Branch/Main to Main
			io.setPoint(26, false);	 // Stop Main (Stop26)
			io.setPoint(27, false);	 // Stop Branch (Stop27)
			io.setPoint(25, true);	 // Allow Goods to go (Stop25)
			smMerge.moveToState(12);		//Goods can go
			break;
		}

		if (io.testToti(11) == true){	//Leave Interloper to be handled by main loop
			smMerge.moveToState(0);
			break;
		}

		if (io.testToti(22) == false){
			display.out("Goods vanished!");
			smMerge.moveToState(0);
			break;
		}
		if (timer1.expired() == true) {
			display.out("Goods held! M11");
			timer1.init(STAYINSTATE);
		}
		break;

	case 12:	//We have committed to allow Goods to go, TOTI22 moving into TOTI20 (TOTI11 if EAST)
		//Give GOODS 20s to move - otherwise if someone else is waiting, it loses its turn

		if (entryFlag){
			timer1.init(STAYINSTATE);
		}

    if (io.testToti(11)){ //Leave Interloper to be handled by main loop
      smMerge.moveToState(0);
      break;
    }
 
 		if (io.testToti(20) == true) {	
			//watch for train moving into T20
			smMerge.moveToState(14);	//Goods going
			break;
		}

		if (io.testToti(22) == false) {	//Goods disappeared without moving into T11/T20
			display.out("Goods vanished!");
			io.setPoint(25, false);	 // Clear the signal (Stop25)
			smMerge.moveToState(0);
			break;
		}
		if (timer1.expired() == true) {
			display.out("Goods stuck T22!");
			if (io.testToti(21) == true){	//Main is waiting
				io.setPoint(25, false);	//Goods can no longer go
				smMerge.moveToState(1);	//go to Main waiting
				break;
			}
			if (io.testToti(21) == true){	//Main is waiting
				io.setPoint(25, false);	//Goods can no longer go
				smMerge.moveToState(1);	//go to Main waiting
				break;
			}
			if (io.testToti(23) == true){	//Branch is waiting
				io.setPoint(25, false);	//Goods can no longer go
				smMerge.moveToState(21);	//go to Branch waiting
				break;
			}
			if (protArea == MERGE) {
				protArea = UNOWNED;  //Give EXIT a chance
			}
			timer1.init(STAYINSTATE); //no-one else waiting, so go round again
		}
		break;

	case 14:	//Train moving, now front is in TOTI20 - only in WEST
		if (entryFlag){	
			timer1.init(STAYINSTATE);
		}
		if (io.testToti(11) == true){
			smMerge.moveToState(15);	//Merge from protected
			break;
		}
		if (io.testToti(20) == false){	
			//if T20 clear without going into T11, we must be backing out...
			smMerge.moveToState(0);
			break;
		}
		if (timer1.expired() == true) {
			display.out("Merge stuck T20!");
			timer1.init(STAYINSTATE);
		}
		break;

	case 15:	//Merge from protected, now front is in TOTI11
		//May be held here by other end's ENTER State Machine at STOP28
		// Stay here until TOTI11 are clear
		if (entryFlag){
      io.setPoint(25, false);  // Clear the signal (Stop25)
		}
   
 		if (io.testToti(11) == false)	{
			smMerge.moveToState(0);	//All done
		}
		break;


		//****************** BRANCH STATES *************************

		//Although there is no BRANCH for the EAST controller, all the logic is here,
		//	so that the code remains identical to MAIN and GOODS

	case 21:	 //Branch waiting
		if (entryFlag){
			timer1.init(STAYINSTATE);
		}

		if (!io.testToti(PROTAREATOTI) && (protArea != EXIT) && !io.testToti(20) && !io.testToti(11)) {	 
			//The way is clear
			if (!digitalRead(eastPin)) {
				protArea = MERGE;	 //no need to take ownership if EAST
			}
//		io.setPoint(20, false);	// No crossover
			io.setPoint(21, false);	// Set Goods/Main point to Main
			io.setPoint(23, true);	// Set Branch/Main to Branch
			io.setPoint(25, false);	 // Stop Goods (Stop25)
			io.setPoint(26, false);	 // Stop Main (Stop26)
			io.setPoint(27, true);	 // Allow Branch to go (Stop27)
			smMerge.moveToState(22);		//Branch can go
			break;
		}

		if (io.testToti(23) == false){
			display.out("Branch vanished!");
			smMerge.moveToState(0);
			break;
		}
		if (timer1.expired() == true) {
			display.out("Branch held! M21");
			timer1.init(STAYINSTATE);
		}
		break;

	case 22:	//We have committed to allow Branch to go, TOTI23 moving into xover and TOTI20
		//Give BRANCH 20s to move - otherwise if someone else is waiting, it loses its turn
	 
		if (entryFlag){
			timer1.init(STAYINSTATE);
		}
	 
    if (io.testToti(PROTAREATOTI) ) { 
			//watch for train moving into crossover
			smMerge.moveToState(24);	//Branch going
			break;
		}

		if (io.testToti(23) == false) {	//Branch disappeared without moving into T11/T12/T20
			display.out("Branch vanished!");
			io.setPoint(27, false);	 // Clear the signal (Stop27)
			smMerge.moveToState(0);
			break;
		}
		if (timer1.expired() == true) {
			display.out("Branch stuck T23!");
			if (io.testToti(21) == true){	//Main is waiting
				io.setPoint(27, false);	//Branch can no longer go
				protArea = UNOWNED;  //Give EXIT a chance
				smMerge.moveToState(1);	//go to Main waiting
				break;
			}
			if (io.testToti(22) == true){	//Goods is waiting
				io.setPoint(27, false);	//Branch can no longer go
				protArea = UNOWNED;  //Give EXIT a chance
				smMerge.moveToState(11);	//go to Goods waiting
				break;
			}
     
			timer1.init(STAYINSTATE); //no-one else waiting, so go round again
		}

		break;

	case 24:	//Train moving, now front is in TOTI12 xover - only in WEST
		if (entryFlag){	
			timer1.init(STAYINSTATE);
		}
		if (io.testToti(20) == true){
			smMerge.moveToState(25);	//Merge from protected
			break;
		}
		if (!io.testToti(PROTAREATOTI)){	
			//if xover clear without going into T20, we must be backing out...
			smMerge.moveToState(0);
			break;
		}
		if (timer1.expired() == true) {
			display.out("Merge stuck T20!");
			timer1.init(STAYINSTATE);
		}
		break;

	case 25:	//Merge from protected, now front is in TOTI20
		if (entryFlag){
      io.setPoint(27, false);  // Clear the signal (Stop27)
		}
		if (!io.testToti(20) && !io.testToti(PROTAREATOTI))	{
			smMerge.moveToState(0);	//All done
		}
    if (io.testToti(11)) {
      smMerge.moveToState(26); //Front moving into T11
    }
		break;

  case 26:  //Merge from protected, now front is in TOTI11
    //May be held here indefinitely by other end's ENTER State Machine at STOP28
    // Stay here until TOTI12, TOTI20 and TOTI11 are clear
    if (entryFlag){
    }
    if (!io.testToti(12) && !io.testToti(20)) {
      if (protArea == MERGE) {    //allow exiting trains as soon as T12, T20 clear
        protArea = UNOWNED;
		io.setPoint(23, false);	//Clear crossover
	  }
      if (!io.testToti(11)) {
        smMerge.moveToState(0); //All done
      }
    }
    break;
    
  default:
    display.out("M-state" + (String)(stripState)+"?!");
    smMerge.moveToState(0); //Illegal state!
    break;
  }
    
}



//================================================================
//							Update the ENTER State Machine
//================================================================


//The ENTER State Machine is controlled by TOTI13, TOTI9 and the TOTIs for each of the
// eight sidings.	The outputs are the siding point settings and STOP Section 28.
//	The ENTER State Machine may also need to take over the Scissor Point scissorsArea
//		in order to put a train into Siding 2.

/*If the EXIT state machine sees T9 occupied and myEnterSiding ==0
then it knows there's a through train that wants out */

void updateEnter() {

	byte state = smEnter.fetch();

	byte stripState = state & 0x7F;
	bool entryFlag = false;

	if (state <128) { //first time in this state
		entryFlag = true;
		smEnter.moveToState(state);	 //set the msb
		if (despatchMode == false) {	//suppress state reporting if we're despatching
			reportStates(false);
		}
	}


	switch (stripState) {

	case 0:	//IDLE
		if (entryFlag) {	//entering state for the first time
			io.setPoint(28, false);	//stop ENTER train until there is a plan
			if ((io.testToti(9) == false) && (io.testToti(13) == false)) {	// T9 and T13 clear, so reset siding points
				io.setPoint(0, true);	//clear siding points
				if (scissorsArea == ENTER) {
					scissorsArea = UNOWNED;
				}
			}
		}

		//We move off IDLE if we see anything in T9 or T13
		if (io.testToti(9)) {	
			smEnter.moveToState(10);	//Entry congested - abnormal, so we need to hold train at Stop 28
			//Only legitimate entry is via occupied T13
			break;
		}
		if (io.testToti(13)) {	//A train has arrived
			smEnter.moveToState(1);	 //train waiting
		}
		break;


	case 1:	//Train in T13, T9 free
    //wait for RFID to be heard - if no RFID, stop at Stop 28
		if (entryFlag){
			timer2.init(STAYINSTATE);
		}

		if (preferredSiding != 0xFF) {    //we've heard an RFID
			myEnterSiding = 0;    //default to THROUGH
			if (preferredSiding == 0xfe) {	//but the RFID wasn't recognised
				display.out("New RFID->THRU!");
			}
			else {	//we have a valid preferred siding
				if (myExitSiding == 0) {	//THROUGH is the ACTIVE despatch, so ignore preferred
					display.out("Queued THROUGH");
				}
				else {
					if (io.testToti(preferredSiding) ) {   //already occupied
						display.out("S" + (String)(preferredSiding) + " is full!");
					} else {	//everyting checks out OK
						myEnterSiding = preferredSiding;
						display.out(trainIdToString(thisTrainRfid) + " to S" + (String)(myEnterSiding));
					}
				}
			}
			preferredSiding = 0xFF;  //only use once
			smEnter.moveToState(11);  //now we know which siding (myEnterSiding)
			break;
		}

		if (!io.testToti(13)) {	//we never heard from the RFID
			io.setPoint(28, false);	//clear everything
			display.out("E vanished(E1)!");
			smEnter.moveToState(0);  //Entering train disappeared!
			break;
		}

		if (timer2.expired() == true) {	//we've waited at Stop 28, but no RFID has been heard, so send through
			display.out("No RFID heard");
			display.out("so go THROUGH!");
			myEnterSiding = 0;    //default to THROUGH
			smEnter.moveToState(11);  //now we know which train it is
		}
		break;

	case 11: //Train in T13, T9 free, train known
		//we may have to wait for train exiting from Siding 1
		//...but if not, we'll go straight to state 12
		if (entryFlag){
			timer2.init(STAYINSTATE);
		}

		if ((scissorsArea != EXIT) || (myEnterSiding != 0 && myEnterSiding != 2)) {
			//then we can go into the siding
			if (myEnterSiding == 0 || myEnterSiding == 2){
				scissorsArea = ENTER; //claim scissors area if necessary
			}
			io.setPoint(28, true);   //let the train go
			setEnterSiding(myEnterSiding);  //set the points
			smEnter.moveToState(12);  //train can go to siding
			break;
		}

		if (!io.testToti(13)) {
			io.setPoint(28, false);	//clear everything
			display.out("E vanished(E11)!");
			smEnter.moveToState(0);  //Entering train disappeared!
			break;
		}

		if (timer2.expired() == true) {
			display.out("Access blocked!");	//We have spent too long in TOTI13
			timer2.init(STAYINSTATE);
		}

		break;

  case 12:  //Train can go into siding, so expecting T9 (or myEnterSiding == 8 && T8)
  
    if (entryFlag){
      timer2.init(STAYINSTATE);
    }

    if (io.testToti(9) || ((myEnterSiding == 8) && io.testToti(8))) {  
      //allow entry to Siding 8 without going into T9 first
      smEnter.moveToState(2);  //train moving to selected siding or THROUGH
      break;
    }
    
    if (!io.testToti(13)) {
      io.setPoint(28, false); //clear everything
      display.out("E vanished(E12)!");
      smEnter.moveToState(0);  //Entering train disappeared!
      break;
    }

    if (timer2.expired() == true) {
      display.out("Stuck to S" + (String)(myEnterSiding) + "E12!"); //We have spent too long in TOTI13
      timer2.init(STAYINSTATE);
      break;
   }

   break;

	case 2: //Train going into siding, or going THROUGH, T9+T13 occupied
		if (entryFlag) {
			timer2.init(STAYINSTATE);
			io.setPoint(28, false);	//stop subsequent trains
		}
		if (io.testToti(myEnterSiding) || (io.testToti(SCISSORSAREATOTI) && (scissorsArea == ENTER))){
			smEnter.moveToState(3);	//front is in siding
			break;
		}

		if (timer2.expired() == true) {
			if (!io.testToti(9) && !io.testToti(SCISSORSAREATOTI)) {
				display.out("E vanished(E2)!");
				smEnter.moveToState(0);  //Entering train disappeared!
				break;
			}
			else {
				display.out("E Stuck(T9+13)!");	//We have spent too long in TOTI9+TOTI13, 
				timer2.init(STAYINSTATE);			// but keep trying
			}
		}
		break;

	case 3: //Train going into siding, T9 occupied
		if (entryFlag) {
			timer2.init(STAYINSTATE);
		}
		if (!io.testToti(9) && (!io.testToti(SCISSORSAREATOTI) || (scissorsArea == EXIT))){  //train cleared TOTI9, or if entering Siding 2,T14 clear too
      //may need to wait for train to complete entering Siding 2
			smEnter.moveToState(0);		//All done
			break;
		}
		if (timer2.expired() == true) {
			io.setPoint(28, false);	//stop subsequent trains
			if ((myEnterSiding == 0) && !io.getPoint(29)) {
				//suppress message if THROUGH is good to go, as it's then an EXIT problem
				display.out("Through waiting!");	//We have spent too long in TOTI9 
			}
			else {
				display.out("E Stuck(T9)!");	//We have spent too long in TOTI9 
			}
			timer2.init(STAYINSTATE);			// but keep trying
		}
		break;

		//Exception states

	case 10:	//Something is sitting at the entrance to the sidings (T9)
		if (entryFlag){
			display.out("E10T9interloper!");
			io.setPoint(28, false);	 //Hold train indefinitely
			break;
		}
		if (io.testToti(9) == false){	//the problem has gone
			smEnter.moveToState(0);
		}
		break;


	default:
		display.out("E-state" + (String)(stripState)+"?!");
		smEnter.moveToState(0);  
		break;			//unrecognised state
	}

}



//================================================================
//							Update the EXIT State Machine
//================================================================

//This pulls a despatch request from the queue and implements it, provided the exit is free
//It needs TOTI10 free if Siding 2...8
//It needs TOTI14 free, and scissorsArea != ENTER if Siding 1
//It needs TOTI12 (Protected Area, WEST only) free if the destination is Main or Goods
//It needs the destination free (MAIN=T19, GODDS=T18, BRANCH=T17)

bool exitAvailable = true;


void updateExit() {

	byte state = smExit.fetch();

	bool entryFlag = false;
	byte stripState = state & 0x7F;

	if (state < 128) { //first time in this state
		entryFlag = true;
		smExit.moveToState(state);	 //set the msb
		if (despatchMode == false) {  //suppress reporting if we're despatching
			reportStates(false);
		}
	}


	switch (stripState) {

	case 0:	//IDLE
		if (entryFlag) {	//entering state for the first time

			if (io.testToti(10) || ((io.testToti(PROTAREATOTI)) && protArea == EXIT)) {
				//don't clear points if there's a train on them
				smExit.moveToState(10);  //Exit congested
				break;
			}
			else {
				io.clearActiveExit();	//forget what we were just doing
				exitSidingPoints(0);	//clear all exit points
				io.setExitModeDisplay(0);	//stop current flashing indication
				io.setPoint(22, false);	//default to MAIN
				io.setPoint(24, false);	//default to Goods/Main
				myExitSiding = 0xff;	//nothing active from the queue

				if (protArea == EXIT) {
					protArea = UNOWNED;
				}

				if (scissorsArea == EXIT) {
					scissorsArea = UNOWNED;
				}
			}
 		}



		if (io.queueNotEmpty()) {	//we have something to do
			myExit = io.getFromQueue();	//this is what we're going to do next
			myExitSiding = myExit & 0x0F;		 //lsn
			exitTrainId = EEPROM.read(StoredTrain + myExitSiding - 1);	//just for info message
			myDestination = myExit & 0x70;		//msn, strip THROUGH flag
			myExitSiding1 = myExitSiding;
			if (myExitSiding1 == 0 ){
				myExitSiding1 = 15;  //we don't want to reset everything for a through train      
			}
			//If it's not a Through train, and the siding is now empty, we've lost the train somehow
			if (myExitSiding != 0) {
				if (!io.testToti(myExitSiding))  {
					display.out("No S" + (String)(myExitSiding)+" to send!");
					smExit.moveToState(10);
					break; //we've lost our train!
				}
			}
			switch (myDestination) {	
			case MAIN:
				smExit.moveToState(2);
				break;
			case GOODS:
				smExit.moveToState(12);
				break;
			case BRANCH:
				smExit.moveToState(22);
				break;
			default:
				display.out("Unknown Exit!");
				break;
			}
		}
		else {	//nothing in the queue
			myExit = 0;
			exitTrainId = 0;
			}
			break;
		
	case 10:	//exit congested, or other exceptional state
		if (entryFlag) {
			timer3.init(STAYINSTATE);
			io.setExitModeDisplay(0);	//stop flashing
		}

		if (!io.testToti(10) && !io.testToti(PROTAREATOTI)  
		    && (!io.testToti(14) || (scissorsArea != EXIT))) {	//train cleared
			//we are all clear, so back to IDLE
			myExit = myExitSiding = myExitSiding1 = myDestination = 0;
			smExit.moveToState(0);
			break;
		}

		if (timer3.expired() == true) {
			String xMsg = "X10 block@T";   //send a message that shows the cause of the blockage
			if (io.testToti(SCISSORSAREATOTI) && (myExitSiding < 2)) xMsg = xMsg + "14;";
			if (io.testToti(10)) xMsg = xMsg + "10;";
			if (io.testToti(PROTAREATOTI)) xMsg = xMsg + "12";
			display.out(xMsg + "!");
			timer3.init(STAYINSTATE);
		}

		break;

//	******	States EXITing to MAIN ******	

	case 2:	//hoping to exit to MAIN
		//here we want to test whether the MAIN onto the display layout is free
		if (entryFlag) {
			timer3.init(STAYINSTATE);	 //wait for route to display layout to become free
		}

		if ((myExitSiding != 0) && (io.testToti(myExitSiding) == false)) {
			display.out("No S" + (String)(myExitSiding)+" to send!");
			smExit.moveToState(10);
		}

		exitAvailable = true;
		if (io.testToti(19)) exitAvailable = false;	//MAIN exit occupied
		if (protArea == MERGE) exitAvailable = false;	//crossover in use for MERGE
		if (io.testToti(PROTAREATOTI)) exitAvailable = false;	//something hogging the crossover
		if (io.testToti(10) && (myExitSiding > 1)) exitAvailable = false;	//something hogging the exit
		if ((myExitSiding == 1)  && ((scissorsArea == ENTER) || io.testToti(14))) exitAvailable = false;	//Siding 1 exit blocked

		if (exitAvailable) {
			if (myExitSiding == 1) {
			   scissorsArea = EXIT;
			}
			//we're good to go
			exitSidingPoints(myExitSiding1);	//set siding exit points
			io.setPoint(22, false);  //Goods/Main to Main
			if (!digitalRead(eastPin)) {   //for WEST only....
				protArea = EXIT;
				io.setPoint(23, false);  //no crossover
				io.setPoint(24, false);  //Branch/Main/Goods to Main/Goods
			}
			io.setExitModeDisplay(myExit);	//committing to exit to MAIN, so set flashing
			smExit.moveToState(3);
			break;
		}
		if (timer3.expired() == true) {
			display.out("Main X busy! X2");
			timer3.init(STAYINSTATE);	 //wait for route to display layout to become free
		}

		break;


	case 3:	//we are allowed to exit to MAIN
		//we are moving when we see train in T10
		if (entryFlag) {
			if (myExitSiding == 0) {
				timer3.init(120);	//allow a full 2 minutes for a THROUGH train
			}
			else {
				timer3.init(STAYINSTATE);
			}
		}

		if (io.testToti(10)) {	//exiting train always goes to TOTI10
			//train moving, so we are now committed
			smExit.moveToState(4);
			break;
		}


		//exceptional actions
		if ((myExitSiding != 0) && !io.testToti(myExitSiding)) {   //if the train in the siding disappears but not via T10
			if ((myExitSiding != 1) || ((myExitSiding == 1) && !io.testToti(14))) {  //Siding 1 may move into T14 before T10
				display.out("No S" + (String)(myExitSiding)+" to send!");
				smExit.moveToState(10);
				break;
			}
		}
		if (timer3.expired() == true) {
			if (myExit & 0x80) { 
				//we can wait for a THROUGH train
				display.out("Wait Through->M");
				timer3.init(STAYINSTATE);
			} else {
				//we never saw the train in T10, so it hadn't moved, even though allowed to
				display.out("Abort send S" + (String)(myExitSiding));
				smExit.moveToState(10);
			}
		}
		break;


	case 4:	//train moving to MAIN, as weve seen it in TOTI10
		//If WEST, look for move into protected area
		//As train is moving, we can no longer timeout and abort
		if (entryFlag) {
			timer3.init(STAYINSTATE);
		}

		if (io.testToti(PROTAREATOTI)) {	//front of train now in protected area (which we own) (WEST only)
			smExit.moveToState(5);
			break;
		}
   
		if (io.testToti(19) == true) {	//train moving into MAIN (this will happen on EAST)
			smExit.moveToState(6);
			break;
		}

		if (!io.testToti(10))	{	//no train exiting
			display.out("S"+(String)(myExitSiding) + " backing? X4");
			smExit.moveToState(10);
			break;
		}

 		if (timer3.expired() == true) {
			display.out("M stuck X4T10!");		//on track, so we have to keep waiting...
			timer3.init(STAYINSTATE);
			break;
		}
		break;


	case 5:	//exiting to MAIN, as weve seen it in ProtArea (WEST only) 
		if (entryFlag) {
			timer3.init(STAYINSTATE);
		}

		if (io.testToti(19) == true) {	//train moving into MAIN
			smExit.moveToState(6);
			break;
		}
		if (!io.testToti(PROTAREATOTI)) {  
			if (io.testToti(10)) {
				display.out("Backing?! X5");
				smExit.moveToState(4);
			} else {
				display.out("Vanished! X5");
				smExit.moveToState(10);
			}
			break;
		}

		if (timer3.expired() == true) {
			display.out("M X stuck X5T20!");		//on track, so we have to keep waiting...
			timer3.init(STAYINSTATE);
		}
		break;


	case 6:	//now actually in MAIN, as we've seen in TOTI19, but still hogging protArea
		// We could be held in this state for some time, waiting for a routing through the display layout
		if (entryFlag) {
		}

		if ((scissorsArea == EXIT) && !io.testToti(SCISSORSAREATOTI)) {		//release scissors area
			scissorsArea = UNOWNED;
		}
		

		if ((!io.testToti(PROTAREATOTI) && !digitalRead(eastPin)) || (!io.testToti(10) && digitalRead(eastPin)) ) {
			//we are out of the protected area, tail moving into MAIN exit
			//we are clear of the shared exit route, but may still be waiting for the display layout
			exitSidingPoints(0); //disable the siding exit
			if (protArea == EXIT) {
				protArea = UNOWNED;
			}
			smExit.moveToState(10);
		}
		break;

		
	//	******	States EXITing to GOODS ******	


	case 12:	//hoping to exit to GOODS
		//here we want to test whether the GOODS onto the display layout is free
		if (entryFlag) {
			timer3.init(STAYINSTATE);	 //wait for route to display layout to become free
		}

		if ((myExitSiding != 0) && (io.testToti(myExitSiding) == false)) {
			display.out("No S" + (String)(myExitSiding)+" to send!");
			smExit.moveToState(10);
		}

		exitAvailable = true;
		if (io.testToti(18)) exitAvailable = false;	//GOODS exit occupied
		if (protArea == MERGE) exitAvailable = false;	//crossover in use for MERGE
		if (io.testToti(PROTAREATOTI)) exitAvailable = false;	//something hogging the crossover
		if (io.testToti(10) && (myExitSiding > 1)) exitAvailable = false;	//something hogging the exit
		if ((myExitSiding == 1)  && ((scissorsArea == ENTER) || io.testToti(14))) exitAvailable = false;	//Siding 1 exit blocked
		if (exitAvailable) {
			if (myExitSiding == 1) {
				scissorsArea = EXIT;
			}
			//we're good to go
			exitSidingPoints(myExitSiding1);  //set siding exit points
			io.setPoint(22, true);  //Goods/Main to Main
			if (!digitalRead(eastPin)) {   //for WEST only....
				protArea = EXIT;
				io.setPoint(23, false);  //no crossover
				io.setPoint(24, false);  //Branch/Main/Goods to Main/Goods
			}
			io.setExitModeDisplay(myExit);	//set flashing
			smExit.moveToState(13);
			break;
		}

		if (timer3.expired() == true) {
			display.out("GoodsX busy!X12");
			timer3.init(STAYINSTATE);	 //wait for route to display layout to become free
		}

		break;


	case 13:	//we are allowed to exit to GOODS
		//we are moving when we see train in T10
		if (entryFlag) {
			if (myExitSiding == 0) {
				timer3.init(120);	//allow a full 2 minutes for a THROUGH train
			}
			else {
				timer3.init(STAYINSTATE);
			}
		}

		if (io.testToti(10))  { //exiting train always goes to TOTI10
			//train moving, so we are now committed
			smExit.moveToState(14);
			break;
		}

		//exceptional actions
		if ((myExitSiding != 0) && !io.testToti(myExitSiding)) {   //if the train in the siding disappears but not via T10
			if ((myExitSiding != 1) || ((myExitSiding == 1) && !io.testToti(14))) {  //Siding 1 may move into T14 before T10
				display.out("No S" + (String)(myExitSiding)+" to send!");
				smExit.moveToState(10);
				break;
			}
		}
		if (timer3.expired() == true) {
		  if (myDestination & 0x80) { 
			//we can wait for a THROUGH train
			display.out("Wait Through->G");
			timer3.init(STAYINSTATE);
		  } else {
			display.out("Abort send S" + (String)(myExitSiding));
			smExit.moveToState(10);
		  }
		}
		break;



	case 14:	//train moving to GOODS, as weve seen it in TOTI10
		//If WEST, look for move into TOTI20
		if (entryFlag) {
		  timer3.init(STAYINSTATE);
		}

		if (io.testToti(PROTAREATOTI)) {  //front of train now in protected area (which we own) (WEST only)
		  smExit.moveToState(15);
		  break;
		}
   
		if (io.testToti(18) == true) {  //train moving into GOODS (this will happen on EAST)
		  smExit.moveToState(16);
		  break;
		}
    
		if (!io.testToti(10))	{	//no train exiting
			display.out("S"+(String)(myExitSiding) + " backing?X14");
			smExit.moveToState(10);
			break;
		}

		if (timer3.expired() == true) {
			display.out("G stuck X14T10!");		//on track, so we have to keep waiting...
			timer3.init(STAYINSTATE);
			break;
		}
		break;


	case 15:	//exiting to GOODS, as weve seen it in ProtArea (WEST only)
		if (entryFlag) {
			timer3.init(STAYINSTATE);
		}

		if (io.testToti(18) == true) {	//train moving into GOODS
			smExit.moveToState(16);
			break;
		}
		if (!io.testToti(PROTAREATOTI)) {  
			if (io.testToti(10)) {
				display.out("Backing?!X15");
				smExit.moveToState(14);
			} else {
				display.out("Vanished!X15");
				smExit.moveToState(10);
			}
			break;
		}

		if (timer3.expired() == true) {
			display.out("G X stuckX15T20!");		//on track, so we have to keep waiting...
			timer3.init(STAYINSTATE);
		}
		break;


	case 16:	//now actually in GOODS, as we've seen in TOTI18, but still hogging protArea
		// We could be held in this state for some time, waiting for a routing through the display layout
		if (entryFlag) {
		}

		if ((scissorsArea == EXIT) && !io.testToti(SCISSORSAREATOTI)) {		//release scissors area
			scissorsArea = UNOWNED;
		}

		if ((!io.testToti(PROTAREATOTI) && !digitalRead(eastPin)) || (!io.testToti(10) && digitalRead(eastPin))) {	//we are out of the protected area, tail moving into GOODS exit
			exitSidingPoints(0); //disable the siding exit
			if (protArea == EXIT) {
				protArea = UNOWNED;
			}
			smExit.moveToState(10);
		}
		break;

//	******	States EXITing to BRANCH ******	


	case 22:	//hoping to exit to BRANCH
		//here we want to test whether the BRANCH onto the display layout is free
		if (entryFlag) {
			timer3.init(STAYINSTATE);	 //wait for route to display layout to become free
		}

		if ((myExitSiding != 0) && (io.testToti(myExitSiding) == false)) {
			display.out("No S" + (String)(myExitSiding)+" to send!");
			smExit.moveToState(10);
		}

		exitAvailable = true;
		if (io.testToti(17)) exitAvailable = false;	//BRANCH exit occupied
		if (io.testToti(10) && (myExitSiding > 1)) exitAvailable = false;	//something hogging the exit
		if ((myExitSiding == 1)  && ((scissorsArea == ENTER) || io.testToti(14))) exitAvailable = false;	//Siding 1 exit blocked
		if (exitAvailable) {
			if (myExitSiding == 1) {
				scissorsArea = EXIT;
			}
			//we're good to go
			exitSidingPoints(myExitSiding1);  //set siding exit points
			io.setPoint(24, true);   //Branch/Main/Goods to Branch
			io.setExitModeDisplay(myExit);	//committing exit to BRANCH, so set flashing
			smExit.moveToState(23);
			break;
		}

		if (timer3.expired() == true) {
			display.out("BranchX busy!X22");
			timer3.init(STAYINSTATE);	 //wait for route to display layout to become free
		}

		break;


	case 23:	//we are allowed to exit to BRANCH
		//we are moving when we see train in T10
		if (entryFlag) {
			if (myExitSiding == 0) {
				timer3.init(120);	//allow a full 2 minutes for a THROUGH train
			}
			else {
				timer3.init(STAYINSTATE);
			}
		}

    if (io.testToti(10))  { //exiting train always goes to TOTI10
			//train moving, so we are now committed
			smExit.moveToState(24);
			break;
		}

		//exceptional actions
		if ((myExitSiding != 0) && !io.testToti(myExitSiding)) {   //if the train in the siding disappears but not via T10
			if ((myExitSiding != 1) || ((myExitSiding == 1) && !io.testToti(14))) {  //Siding 1 may move into T14 before T10
				display.out("No S" + (String)(myExitSiding)+" to send!");
				smExit.moveToState(10);
				break;
			}
		}
		if (timer3.expired() == true) {
		  if (myDestination & 0x80) { 
			//we can wait for a THROUGH train
			display.out("Wait Through->B");
			timer3.init(STAYINSTATE);
		  } else {
			  //we never saw the train in T10, so it hadn't moved, even though allowed to
			  display.out("Abort send S" + (String)(myExitSiding));
			smExit.moveToState(10);
		  }
		}
		break;


	case 24:	//train moving to BRANCH, as weve seen it in TOTI10
    //If WEST, BRANCH exit does NOT go into ProtArea!
    if (entryFlag) {
      timer3.init(STAYINSTATE);
    }

    if (io.testToti(17) == true) {  //train moving
      smExit.moveToState(26);
      break;
    }
 
 		if (!io.testToti(10))	{	//no train exiting
			display.out("S"+(String)(myExitSiding) + " backing?X24");
			smExit.moveToState(10);
			break;
		}

		if (timer3.expired() == true) {
			display.out("B stuck X24T10!");		//on track, so we have to keep waiting...
			timer3.init(STAYINSTATE);
			break;
		}
		break;

	case 26:	//now actually in BRANCH, as we've seen in TOTI17
		// We could be held in this state for some time, waiting for a routing through the display layout
		if (entryFlag) {
		}

		if ((scissorsArea == EXIT) && !io.testToti(SCISSORSAREATOTI)) {		//release scissors area
			scissorsArea = UNOWNED;
		}

		if (!io.testToti(10)) {	//we are clear of the shared exit route, but may still be waiting for the display layout
			exitSidingPoints(0); //disable the siding exit
			smExit.moveToState(10);
		}
		break;

	default:
		display.out("X-state" + (String)(stripState)+"?!");
		smExit.moveToState(0);  
		break;      //unrecognised state
    
	}
}

		//================================================================
		//							Code clusters
		//================================================================


void reportStates(bool roFlag) {
	//show all three state engine states "Mnn Enn Xnn (rf)"
	//if RFID is going to be ignored, show "Mnn Enn Xnn -rf-"

  String allStates = "$";
	allStates += "M" + byteToString(smMerge.fetch());
	allStates += " E" + byteToString(smEnter.fetch());
  byte exitState = smExit.fetch();
	allStates += " X" + byteToString(exitState);
  switch(exitState & 0x7F) {
    case 0:
      break;  //idle, so no train
    case 10:
      allStates += "(**)";  //anomalous, so no idea which train!
      break;
    default: 
     allStates += trainIdToString(exitTrainId);
	 if (roFlag) {	//show RFID as -xx- if not going to be saved
		 allStates.replace("(", "-");
		 allStates.replace(")", "-");
	 }
     break;
  } 

	display.out(allStates);

}

String trainIdToString (byte t) {
  //Display train ID as (hex)
    if (t == 0xFF) {
      return("(?\?)");  //avoid the trigraph (who uses those anyway?)
    } else {
      return("(" + byteToHexString(t) + ")");
    }
}

String byteToHexString(int s) {
	//return a byte (7bits) as a string of two digits
	String myString = String(s,HEX);	//remove first time flag
	if (myString.length() == 1) {
		myString = "0" + myString;
	}
	return(myString);
}

String byteToString(int s) {
  //return a byte (7bits) as a string of two digits
  String myString = String(s & 0x7F); //remove first time flag
  if (myString.length() == 1) {
    myString = "0" + myString;
  }
  return(myString);
}

void enterRunMode(){
	//do this whenever we enter Run mode from wherever
	
	beeper.out(100);
	delay(100);	//I'm not sure why we need this!
	testMode = 0;
	despatchMode = false;
 
  display.init(DEBUG);  //just in case the display has got screwed

	display.out("Run Mode");
	reportStates(false);
}


void setEnterSiding(byte siding) {
  if (siding != 0xFF) {
    for (int setSiding = 8; setSiding > 0; setSiding--) { //now set just the point for the siding we want
      if (siding == setSiding) {
        io.setPoint(setSiding, true);
      }
      else {
        io.setPoint(setSiding, false);   //and clear all the others
      }
    }
  }
}


void exitSidingPoints(byte siding){
	//select the exit siding points for whichever siding is specified, and allow train to go
	// Clear all the others
	// Through = siding 15 
	// To clear all exit points and hold the THROUGH signal, exitSidingPoints(0)
	byte siding1 = siding & 0x0F;	 //strip destination

	if (siding1 == 1) {	//this is a special case, as it relies on the scissor points
		io.setPoint(2, false);	//clear entry crossover
	}

	//clear all exit points 9...16 except the one required
	for (int setExitPoints = 9; setExitPoints < 17; setExitPoints++){
		if (siding1 == setExitPoints - 8) {
			io.setPoint(setExitPoints, true);
		}
		else {
			io.setPoint(setExitPoints, false);
		}
	}
 
	if ((siding1 > 1) && (siding1 < 9)) {	//set exit point, unless siding 1 or THROUGH
		io.setPoint(17, true);	 //allow sidings 2...8 to exit
		io.setPoint(29, false);	//stop THROUGH train if any
	}
 	else {	 //sidings 0 (reset),1,15 (through)
		 io.setPoint(17, false);	//clear point for THROUGH/siding 1
		if (siding1 == 0 ) {  //reset
			io.setPoint(29, false);	 //reset THROUGH/siding1 exit too
		}
		else {
			io.setPoint(29, true);	//allow THROUGH and siding 1 to pass Stop29
		}
	}

}

int findNextSiding(int siding, bool up){
	//find the next available siding that is occupied and not yet queued, display and return the result
	//if no alternatives, return with what we started with
  //if nothing to queue, display error message and return -1
	//The range for siding is 0...8 (0=Through)

  //check 8 sidings and Through, starting with 'siding' and wrapping round if necessary
  int newSiding = -1;
  int checkSiding;
  checkSiding = siding & 0x0F;
  for (int scrollLoop = 0;scrollLoop < 10;scrollLoop++) {
    if (up) {
      if (++checkSiding == 9) checkSiding = 0;
    }else {
      if (--checkSiding == -1) checkSiding = 8;
    }
    if (io.testToti(checkSiding) || (checkSiding == 0)) { //there is a train, or it's THROUGH
      if (io.isThisSidingQueued(checkSiding) == false) { //it has not already been queued
        newSiding = checkSiding;
        break;
      }
    }
  }
	if (newSiding == -1){
		display.out("Nothing to queue");
	} else {
    display.out(sidingString(newSiding));
  }
	return newSiding;
}

String sidingString(byte siding){
	//return the name of the siding and the RFID of whatever lives there
	if (siding == 0) {
		return("Through");
	}
	else {
    return("Siding " + (String)siding + trainIdToString(EEPROM.read(StoredTrain + siding - 1)));
	}
}

byte hexStrToByte(String toConvert) {
  //convert a hexadecimal number as the last 2 chars of string into a byte
  // NB Invalid hex digits will not be trapped!
  char msn = toConvert.charAt(toConvert.length()-2);
  char lsn = toConvert.charAt(toConvert.length()-1);
  return(charToNib(msn) * 16 + charToNib(lsn)); 
}

byte charToNib(char toConvert) {
	if (toConvert >= 'a') return (toConvert - 'a' + 10);
	if (toConvert >= 'A') return (toConvert - 'A' + 10);
	return (toConvert - '0');
}
