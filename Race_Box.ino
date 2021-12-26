/***************************************************************
* Slotcar	Racing Computer 
* Author:	Stefan Siegel
* License:GNU General Public License v3.0 (see repo link below)
* GitHub:	https://github.com/ssie4273/SlotcarComputer.git
****************************************************************/
#define BOUNCE_WITH_PROMPT_DETECTION

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>

// I2C Display
// I2C Adresse: 0x27
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
 
// 16 bit Portexpander LED Startampel
// I2C Adresse> 0x20
const uint8_t MCP_ADDRESS = 0x20;
const uint8_t MCP_GPIOA = 0x12;
const uint8_t MCP_GPIOB= 0x13;
const uint8_t MCP_IODIRA = 0x00;
const uint8_t MCP_IODIRB = 0x01;

// I2C Memory 256k
// I2C Adresse: 0x50

// Pinbelegung: Taster u. Schalter 
const int buzzer = 10;
const int taste1=3;
const int taste2=4;
const int taste3=5;
const int taste4=6;
// Entprellen:
Bounce btn1 = Bounce(taste1,30);
Bounce btn2 = Bounce(taste2,30);
Bounce btn3 = Bounce(taste3,30);
Bounce btn4 = Bounce(taste4,30);
const int set_race=7;
const int rel_A=8;
const int rel_B=9;

// Pinbelegung: IR Lichtschranke
const int IR_A = 1; // Bahn A
const int IR_B = 0; // Bahn B


// Schalter: Race -- Set
boolean settingsSwitch = false;
boolean raceOn = false; // Rennen laeuft.

// Parameter evtl. anpasssen:	
const long penalty = 3000;			// Zeitstrafe bei Fehlstart (ms) 
unsigned long previousRaceDuration = 0;
unsigned long startTimeRace;	// Zeitstempel: Start des Rennens
const int schwellwert_A = 20; // IR Empfindlichkeit Bahn A
const int schwellwert_B = 50; // IR Empfindlichkeit Bahn B
const int IR_sensor_speed = 5;// IR Brücke wird alle IR_sensor_speed ms ausgelesen
const int IR_off_cycle = 350; // IR Brücke wird für IR_out_cycle * IR_sensor_speed nicht ausgelesen: nur Spitze des Fahrzeugs wird gewertet.
const int gesamt = 1000;		// Signalabstand der LED Startampel (ms)
const int tone_short = 50;
const int tone_long = 300;

// nicht anpassen:
int analog_A; // ausgelesener Wert Lichtschranke Bahn1
int analog_B; // ausgelesener Wert Lichtschranke Bahn2
int diffMax_A = 0; 
int diffMax_B = 0;
int vorigeMessung_A, vorigeMessung_B, diff_A, diff_B;

// smoothing der analogRead Werte:
const int numReadings = 10;			// roundrobin nach 10 Werten
int readingsA[numReadings];     // the readings from the analog input
int readingsB[numReadings];     // the readings from the analog input
int readIndex = 0;
int averageA, averageB;					// Mittelwert der letzen <numReadings> Analogwerte
int totalA = 0;									// Summe der Analogwerte
int totalB = 0;

int n_A = IR_off_cycle;	// Zaehlvariable beim augeschalteter IR Bruecke				
int n_B = IR_off_cycle;
boolean crossingIR_A = false;
boolean crossingIR_B = false; // Flags f. Durchfahrt durch IR Schranke
boolean fehlstart_A = false; // Flags f. Fehlstart
boolean fehlstart_B = false;
int offset = 0; // LED Bitmuster: offset 6,12 fuer Bahn A fehlstart oder Bahn B Fehlstart

// Parameter f. Zeitmessung:
int runde_A = 0;
int besteRunde_A = 0;
unsigned long myTime_A;
unsigned long startTime_A = 0;
float lapTime_A, besteZeit_A;
int runde_B = 0;
int besteRunde_B = 0;
unsigned long myTime_B;
unsigned long startTime_B = 0;
float lapTime_B, besteZeit_B;

// Rennparameter default:
int rundenAnzahl = 10;
int rennDauer = 5; // in Minuten
const int deltarennDauer = 1; 
const int deltarundenAnzahl = 5;
const int maxrennDauer = 15; 
const int minrennDauer = 2;
const int maxrundenAnzahl = 100;
const int minrundenAnzahl = 5;
// Rennmodus true: Zeitrennen, false: Rundenrennen
boolean rennModusZeit = true; 

// Sonderzeichen f. LED Display
byte ArrowDown[] = {
  B00000,
  B00100,
  B00100,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100
};

void readoutLanes(){
	// beide Bahnen auslesen und Differenz speichern
	analog_A = analogRead(IR_A);
  analog_B = analogRead(IR_B);
  // Debugging:
  //  Serial.print(analog_A); Serial.print(" ("); Serial.print(vorigeMessung_A);Serial.print(")");
  //  Serial.print("    ");
  //  Serial.print(analog_B); Serial.print(" ("); Serial.print(vorigeMessung_B);Serial.println(")");
  totalA = totalA - readingsA[readIndex];
  totalB = totalB - readingsB[readIndex];
	readingsA[readIndex] = analog_A;
	readingsB[readIndex] = analog_B;
  totalA = totalA + readingsA[readIndex];
  totalB = totalB + readingsB[readIndex];
	readIndex = readIndex + 1;
	if (readIndex >= numReadings) readIndex = 0; // wrap around
	averageA = totalA / numReadings;
	averageB = totalB / numReadings;
		
	diff_A = abs(analog_A - averageA);
  diff_B = abs(analog_B - averageB);
	if (diff_A < schwellwert_A) crossingIR_A = false; else crossingIR_A = true; // false setzt es auf jeden Fall zurueck	
  if (diff_B < schwellwert_B) crossingIR_B = false; else crossingIR_B = true;
}

void showDisplay(int zeile, int spalte, String text) {
	// vereinfachte Anzeige auf LCD 
	lcd.setCursor(spalte,zeile);
	lcd.print(text);
}

void writeRegister(uint8_t address, uint8_t ledBitmuster) {
// Startampel, I2C PortExpander ansteuern:
  Wire.beginTransmission(MCP_ADDRESS);
  Wire.write(address);
  Wire.write(ledBitmuster);
	Wire.endTransmission();
}

void startTone(int toneheigth, unsigned long intervalTone, int versatz, uint8_t portexpanderA, uint8_t portexpanderB){
	// Diese Funktion muss genau 1 Sekunde laufen (das triggert das Weiterschalten der Startampel)
	// Tonabspielen und gleichzeitig IR Schranken auslesen
	// Timer fuer Ton und Lichtschranke; Werte in Millisekunden
	unsigned long previousMillisLoop= 0; // Gesamtloop --> 1 sec, Schaltintervall fuer Startampel
	unsigned long previousMillisTone = 0; // Tone: Buzzer
	unsigned long previousMillisLED = 0; // LED: Blinktimer fuer Fehlstart LED
	unsigned long previousMillisIr = 0;		// Ir: IR Bruecke auslesen
	unsigned long currentMillisLoop;
	boolean redLedOn = true;
	int8_t toggle = 0b11111111;
	//int8_t portUNDbitmuster;
	const long intervalLoop = 1000; // 1000 ms pro Anzeige 
	const long intervalLED = 200; // Blinkintervall fuer Fehlstart LED in ms 
	const long intervalIr = (long) IR_sensor_speed; // aus Variablen Definition 

	readoutLanes();
	boolean buzzerHasPlayed = true;
	boolean loopActive = true;

	currentMillisLoop=millis();
	if (toneheigth > 0) {
		tone(buzzer,toneheigth);
		delay((int)intervalTone);
	}	

	previousMillisLoop = currentMillisLoop;
	while (loopActive) {
		currentMillisLoop=millis();
		
		if (!buzzerHasPlayed){
			tone(buzzer,toneheigth);
			buzzerHasPlayed = true;
		}

		if (currentMillisLoop - previousMillisLoop >= intervalLoop) {
			// Abbruchbedingung nach intervalLoop setzen
			previousMillisLoop = currentMillisLoop;
			loopActive = false;
		} 

		// Ton aus nach intervalTone 
		unsigned long currentMillisTone=millis();
		if (currentMillisTone - previousMillisTone >= intervalTone){
			previousMillisTone = currentMillisTone;
			noTone(buzzer);
		} 
		// IR auslesen und bei Durchfahrt (= Fehlstart) offset setzen bzw. Bahnstrom abschalten
		unsigned long currentMillisIr = millis();
		if (currentMillisIr - previousMillisIr >= intervalIr){
			previousMillisIr = currentMillisIr;
			readoutLanes();
			
			if (crossingIR_A) {
				digitalWrite(rel_A, LOW); // Relais Bahn A abschalten
				fehlstart_A = true;
				if (versatz <= 6) versatz = 6; // linke rote LED an
					else versatz = 18; // beide Aussen LED  --> rot
			} 	
			if (crossingIR_B) {
				digitalWrite(rel_B, LOW); // Relais Bahn A abschalten
				fehlstart_B = true;
				if (versatz == 0) versatz = 12; // beide Aussen LED -->rot 
						else if (versatz == 6) versatz = 18;//rechte rote LED an
			} 	
		}
		// Fehlstart LED blinken
		unsigned long currentMillisLED = millis();
		if (currentMillisLED - previousMillisLED >= intervalLED){
			previousMillisLED = currentMillisLED;
			// toogle red LED
			if (redLedOn) {
				toggle = 0b11111101; 
				redLedOn = !redLedOn;
			} else {
				toggle = 0b11111111;
				redLedOn = !redLedOn;
			}
			// Bahn A oder Bahn B Fehlstart --> Portexpander Register A oder B:
			if (versatz == 6)  writeRegister(MCP_GPIOA, toggle & portexpanderA);	else if (versatz == 12) writeRegister(MCP_GPIOB, toggle & portexpanderB); 
		}		
		previousMillisTone=millis();
	} // end while. solange wie intervalLoop 
	offset = versatz;
} 
	
void startingSignal(){
	// Startampel Licht- und Tonsignal definieren
	// in den Arrays: 6 Bitmuster normal, 6 Bitmuster Fehlstart A, 6 Bitmuster Fehlstart B, 6 Bitmuster Fehlstart A+B
	uint8_t Port_A[24] = {
		0b00000001,0b00000101,0b00010101,0b01010101,0b01010101,0b00000000,
		0b00000010,0b00000110,0b00010110,0b01010110,0b01010110,0b00000010,
		0b00000001,0b00000101,0b00010101,0b01010101,0b01010101,0b00000000,
		0b10101010,0b10101010,0b10101010,0b10101010,0b10101010,0b10101010
	};
	uint8_t Port_B[24] = {
		0b00000000,0b00000000,0b00000000,0b00000000,0b00000001,0b00000000,
		0b00000000,0b00000000,0b00000000,0b00000000,0b00000001,0b00000000,
		0b00000010,0b00000010,0b00000010,0b00000010,0b00000010,0b00000010,
		0b00000010,0b00000010,0b00000010,0b00000010,0b00000010,0b00000010
	};
	int tone_frequency[6] = {0,1000,1000,1000,1000,2000};
	int tone_duration[6] = {0,tone_short,tone_short,tone_short,tone_short,tone_long};
	int i = 0;
	// Zeige Renninfo auf Display
	// Anzeige: Bereit?, Bestaetigen mit taste1
	// 5 Sekunden Countdown auf Display, Ton (buzzer)
	// s.a. https://electric-junkie.de/2020/08/arduino-millis-anstatt-von-delay/
	
	readoutLanes();
	// very first LED.
	delay(1000);
	int blink=100;
	for (int i = 0; i <= 2; i++) { 
		writeRegister(MCP_GPIOA,1);
		writeRegister(MCP_GPIOB,2);
		delay(blink);
		writeRegister(MCP_GPIOA,2);
		writeRegister(MCP_GPIOB,1);
		delay(blink);
	}
		writeRegister(MCP_GPIOA,1);
		writeRegister(MCP_GPIOB,2);
		delay(blink);
		writeRegister(MCP_GPIOA,2);
		writeRegister(MCP_GPIOB,0);
		delay(blink);
	// now start the signal:	
	i = 0;	
	while (i < 6) {
		int index = i + offset;
		// Serial.print(index); Serial.print("  "); Serial.println(Port_A[index],BIN);
		// Serial.print(index); Serial.print("  "); Serial.println(Port_B[index],BIN);
		startTone(tone_frequency[i],(unsigned long)tone_duration[i],offset,Port_A[index],Port_B[index]);
		writeRegister(MCP_GPIOA, Port_A[index]);
		writeRegister(MCP_GPIOB, Port_B[index]);
		i++;
	};
}
 
void raceLoop() {
  // aktives Rennen
	readoutLanes(); // IR Bruecken lesen
  if (crossingIR_A) {
    //Auto A durchfährt Lichtschranke
    if (n_A == IR_off_cycle) { // nur beim allererstem Signal triggern, danach stummschalten fuer IR_offcycle*IR_sensor_speed
			if (runde_A > 0) {
				myTime_A = millis();
				lapTime_A = float(myTime_A - startTime_A) / 1000;
				if (lapTime_A < besteZeit_A && runde_A > 0) {
					besteZeit_A = lapTime_A;
					besteRunde_A = runde_A;
				}
				//Serial.print("A: "); Serial.print("Runde: ");Serial.print(runde_A);Serial.print("  Zeit: ");Serial.println(lapTime_A,3);
				//Serial.print("A:  best: ");Serial.print(besteRunde_A);Serial.print("  Zeit: ");Serial.println(besteZeit_A,3);
				//Serial.println();
				showDisplay(1,0,"          ");
				showDisplay(1,0,(String)runde_A);
				double t = (double)lapTime_A;
				lcd.setCursor(4,1);
				lcd.print(t,2);
				showDisplay(2,0,"--- beste Runde: ---");
				showDisplay(3,0,"          ");
				showDisplay(3,0,(String)besteRunde_A);
				t = (double)besteZeit_A;
				lcd.setCursor(4,3);
				lcd.print(t,2);
      }
      runde_A++;
      n_A--;
      //neue startTime festlegen:
      startTime_A = myTime_A;
    }
  }
  if (crossingIR_B) {
    // Auto B durchfährt Lichtschranke
    if (n_B == IR_off_cycle) { // nur beim allererstem Signal triggern, danach stummschalten fuer IR_offcycle*IR_sensor_speed
      if (runde_B > 0) {
				myTime_B = millis();
				lapTime_B = float(myTime_B - startTime_B) / 1000;
				if (lapTime_B < besteZeit_B && runde_B > 0) {
					besteZeit_B = lapTime_B;
					besteRunde_B = runde_B;
				}
				//Serial.print("B: "); Serial.print("Runde: ");Serial.print(runde_B);Serial.print("  Zeit: ");Serial.println(lapTime_B,3);
				//Serial.print("B:  best: ");Serial.print(besteRunde_B);Serial.print("  Zeit: ");Serial.println(besteZeit_B,3);
				//Serial.println();
				showDisplay(1,11,"          ");
				showDisplay(1,11,(String)runde_B);
				double t = (double)lapTime_B;
				lcd.setCursor(15,1);
				lcd.print(t,2);
				showDisplay(2,0,"--- beste Runde: ---");
				showDisplay(3,11,"          ");
				showDisplay(3,11,(String)besteRunde_B);
				t = (double)besteZeit_B;
				lcd.setCursor(15,3);
				lcd.print(t,2);
      }
      runde_B++;
      n_B--;
      //neue startTime festlegen:
      startTime_B = myTime_B;
    }
  } // Ende crossingIR_B

  // IR Brücke evtl. zurücksetzen und IR Messung für Vergleich speichern
  if (n_A < IR_off_cycle) n_A--;
  if (n_A == 0) n_A = IR_off_cycle;
  if (n_B < IR_off_cycle) n_B--;
  if (n_B == 0) n_B = IR_off_cycle;
  // IR Sensoren erneut auslesen.
  delay(IR_sensor_speed);
} // end of raceLoop

void defineSettings(){
	boolean firstPageReady = false;
	boolean secondPageReady = false;
  lcd.createChar(0, ArrowDown);
  settingsSwitch=digitalRead(set_race);
	while (settingsSwitch) {
		// 1. Seite: Zeitrennen oder Rundenrennen
		lcd.clear();
		showDisplay(0,0,"Einstellungen:");
		showDisplay(1,0,"Renntyp:");
		showDisplay(2,0,"Zeit          Runden");
		lcd.setCursor(1,3);
		lcd.write(0); // Pfeil nach unten 
		lcd.setCursor(18,3);
		lcd.write(0); // Pfeil nach unten 
		while (!firstPageReady){
			btn1.update();
			btn4.update();
			if (btn1.rose()) {
				rennModusZeit=true;	// Zeitrennen
				firstPageReady = true;
			}
			if (btn4.rose()) {
				rennModusZeit=false; // Rundenrennen
				firstPageReady=true;
			}
		} // end of firstPageReady
		// 2. Seite: Anzahl Minuten oder Rundeneinstellen:
		lcd.clear();
		showDisplay(0,0,"Einstellungen:");
		showDisplay(1,0,"Renntyp:");
		if (rennModusZeit) showDisplay(1,8,"Zeitrennen"); else showDisplay(1,8,"Rundenrennen");
		delay(10);
		while (!secondPageReady) {
		// Zeit oder Runcenanzahl einstellen:
			btn2.update();
			btn3.update();
			btn4.update();
			if (rennModusZeit) { // Zeitrennen: adjust Parameter
				showDisplay(3,6,"-");
				showDisplay(2,0,"                    ");
				showDisplay(2,9,(String)rennDauer);
				showDisplay(3,13,"+");
				if (btn2.rose()) {
					if (rennDauer - deltarennDauer >= minrennDauer) rennDauer = rennDauer - deltarennDauer;
				} 
				if (btn3.rose()) {
					if (rennDauer + deltarennDauer <= maxrennDauer) rennDauer = rennDauer + deltarennDauer; 
				}
			} // End Zeitrennen: Adjust Parameter
			else { // Rundenrennen: adjust Parameter
				showDisplay(3,6,"-");
				showDisplay(2,0,"                    ");
				showDisplay(2,9,(String)rundenAnzahl);
				showDisplay(3,13,"+");
				if (btn2.rose()) {
					if (rundenAnzahl - deltarundenAnzahl >= minrundenAnzahl) rundenAnzahl = rundenAnzahl - deltarundenAnzahl;
				} 
				if (btn3.rose()) {
					if (rundenAnzahl + deltarundenAnzahl <= maxrundenAnzahl) rundenAnzahl = rundenAnzahl + deltarundenAnzahl; 
				}
			}// End Rundenrennen: adjust Parameter
			
			showDisplay(3,17,"OK");
			if (btn4.rose()){
				secondPageReady = true;
			}
		} // end od secondPageReady
		// Alle Settings verfuegbar:
		lcd.clear();
		showDisplay(0,0,"Renntyp:");
		if (rennModusZeit) {
			showDisplay(0,8,"Zeitrennen");
			showDisplay(1,0,"Gesamtzeit:");
			showDisplay(1,11,(String)rennDauer);
		} 
		else {
			showDisplay(0,8,"Rundenrennen");
			showDisplay(1,0,"Gesamtrunden:");
			showDisplay(1,13,(String)rundenAnzahl);
		}
		showDisplay(2,1,"neu");
		lcd.setCursor(1,3);
		lcd.write(0); // Pfeil nach unten 
		while (secondPageReady && settingsSwitch){ // alles Settings gesetzt und Schalter noch auf Settings
			btn1.update();
			if (btn1.rose()) {
				firstPageReady = false;	
				secondPageReady = false;	
			}
		settingsSwitch=digitalRead(set_race); // Schalter noch auf Settings?
		} 	
	} // end of while(settingsSwitch)
} // end of define Settings
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);	// open serial interface
	Wire.begin();					// open I2C bus
  
	lcd.init();
  lcd.backlight();
	showDisplay(0,0,"*** RaceComputer ***");
  besteZeit_A = 10000000.0;
  besteZeit_B = 10000000.0;
	
	// Smoothing values reported by IR bridges. Pre-set readings arrays.
	for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsA[thisReading] = analogRead(IR_A);
    readingsB[thisReading] = analogRead(IR_B);
		totalA = totalA + readingsA[readIndex];
		totalB = totalB + readingsB[readIndex];
		//Serial.print(readingsA[thisReading]);Serial.print("::::");
		//Serial.println(readingsB[thisReading]);
  }
	averageA = totalA / numReadings;
	averageB = totalB / numReadings;
 
  // alle Expander ports: OUTPUT, rot-gruen Testblinken
  writeRegister(MCP_IODIRA, 0b00000000);
  writeRegister(MCP_IODIRB, 0b00000000);
	int blink = 150;
	int b  = 0b0000000000000001;
	int hm = 0b1111111100000000;
	int lm = 0b0000000011111111;
	int portA,portB;
	//Lauflicht gruen von links nach rechts:
	for (int i=1; i <= 5; i++) {
		portA = b&lm;
		portB = (b & hm) >> 8;
		writeRegister(MCP_GPIOA,portA);
		writeRegister(MCP_GPIOB,portB);
		//Serial.print("PORT>  ");Serial.print(portB,BIN); Serial.print("  ");Serial.println(portA,BIN);
		b <<= 2 ; //von low nach high 
		delay(blink);
	};
	// Lauflicht rot von rechts nach links
	b <<= 1; // eins weiter auf rot schieben.
	for (int i=1; i <= 6; i++) {
		portA = b&lm;
		portB = (b & hm) >> 8;
		writeRegister(MCP_GPIOA,portA);
		writeRegister(MCP_GPIOB,portB);
		//Serial.print("PORT>  ");Serial.print(portB,BIN); Serial.print("  ");Serial.println(portA,BIN);
		b >>= 2 ; //von high nach low
		delay(blink);
	};
	//LEDs aus:
	writeRegister(MCP_GPIOA,0);
	writeRegister(MCP_GPIOB,0);

	// Definition Taster u. Schalter:
  pinMode(taste1,INPUT);
  pinMode(taste2,INPUT);
  pinMode(taste3,INPUT);
  pinMode(taste4,INPUT);
  pinMode(set_race,INPUT);
  pinMode(rel_A,OUTPUT);
  pinMode(rel_B,OUTPUT);
  
  // set Bahnrelais to HIGH (Fahrstrom ein, Kontroll LED auf Modul sind aus):
  digitalWrite(rel_A, HIGH);
  digitalWrite(rel_B, HIGH);
} // void setup

void loop() {
	// put your main code here, to run repeatedly:
	settingsSwitch=digitalRead(set_race);
	if (settingsSwitch) {
		defineSettings();
	}
	else { // Switch steht auf "Rennen" 
		offset=0; // LED: normale Startsequenz
		lcd.clear();	
		if (rennModusZeit) {
			showDisplay(0,1,"Zeit: ");
			showDisplay(0,7,(String)rennDauer);
			showDisplay(0,10,"Min.");
		} else {
			showDisplay(0,1,"Runden: ");
			showDisplay(0,9,(String)rundenAnzahl);
			showDisplay(0,12,"R.");
		}
		// Bahnstrom einschalten:
		digitalWrite(rel_A, HIGH); 
		digitalWrite(rel_B, HIGH);
		startingSignal(); 	
		startTimeRace = millis(); // Zeitstempel: Start des Rennens 
		startTime_A = startTimeRace;
		startTime_B = startTimeRace;
		raceOn = true;
		while ((!settingsSwitch && raceOn)) {// fehlen noch Endebedingungen (Rundenzahl erreicht, Zeit erreicht.
			// Fehlstartstrafe wieder freischalten:
			if ((fehlstart_A || fehlstart_B) && (millis()-startTimeRace > penalty)) {
				digitalWrite(rel_A,HIGH);
				digitalWrite(rel_B,HIGH);
				fehlstart_A = false;
				fehlstart_B = false;
				writeRegister(MCP_GPIOA,0);
				writeRegister(MCP_GPIOB,0);
			}; 
			raceLoop();
			// Abbruch pruefen
			float aktuelleRenndauer = float(millis()-startTimeRace)/1000;
			float r = float(rennDauer)*60;
			Serial.print(aktuelleRenndauer); Serial.print(" ---  "); Serial.println(r);
			if ( !rennModusZeit && ((runde_A > rundenAnzahl) || (runde_B > rundenAnzahl)) ) {
				// Race ENDE
				raceOn = false;
				digitalWrite(rel_A, LOW); 
				digitalWrite(rel_B, LOW);
				showDisplay(3,0,"ENDE");
				delay(60000);	
			}
			if ( rennModusZeit && (aktuelleRenndauer > r) ) {
				// Race ENDE
				raceOn = false; 
				digitalWrite(rel_A, LOW); 
				digitalWrite(rel_B, LOW);
				showDisplay(3,0,"ENDE");
				delay(60000);	
			}
			settingsSwitch=digitalRead(set_race); // Schalter neu einlesen.
		} 
	} // Ende von else Rennen
}
