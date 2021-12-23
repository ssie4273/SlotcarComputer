// $Rev: 14 $
// $Author: stefan $
// $Date: 2021-12-05 21:45:05 +0100 (Sun, 05 Dec 2021) $
// $Header: file:///home/stefan/Documents/Carrera/repository/Race_Box/Race_Box.ino 14 2021-12-05 20:45:05Z stefan $
#define BOUNCE_WITH_PROMPT_DETECTION

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>

// I2C Display
// I2C Adresse: 0x27
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
 
//Portexpander LED Startampel
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

// Parameter evtl. anpasssen:
const int schwellwert_A = 20; // IR Empfindlichkeit Bahn A
const int schwellwert_B = 50; // IR Empfindlichkeit Bahn B
<<<<<<< Updated upstream
const int IR_sensor_speed = 5; // IR Brücke wird alle IR_sensor_speed ms ausgelesen
=======
const int IR_sensor_speed = 10; // IR Brücke wird alle IR_sensor_speed ms ausgelesen
>>>>>>> Stashed changes
const int IR_off_cycle = 350; // IR Brücke wird für IR_out_cycle * IR_sensor_speed nicht ausgelesen: nur Spitze des Fahrzeugs wird gewertet.
const int gesamt = 1000;
const int tone_short = 50;
const int tone_long = 300;

// nicht anpassen:
int analog_A; // ausgelesener Wert Lichtschranke Bahn1
int analog_B; // ausgelesener Wert Lichtschranke Bahn2
int diffMax_A = 0; 
int diffMax_B = 0;
int vorigeMessung_A, vorigeMessung_B, diff_A, diff_B;
int diffMax_A = 0;
int diffMax_B = 0;
int n_A = IR_off_cycle;
int n_B = IR_off_cycle;
<<<<<<< Updated upstream
boolean crossingIR_A, crossingIR_B; // Flags f. Fehlstart
=======
boolean crossingIR_A, crossingIR_B;
>>>>>>> Stashed changes
int offset = 0; // offset 6,12 fuer Bahn A fehlstart oder Bahn B Fehlstart

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
	const int sensorMax_A = 150;
	analog_A = constrain(analog_A, 0,sensorMax_A);
	analog_A = map(analog_A,0,sensorMax_A,0,255);
  // Debugging:
  //  Serial.print(analog_A); Serial.print(" ("); Serial.print(vorigeMessung_A);Serial.print(")");
  //  Serial.print("    ");
  //  Serial.print(analog_B); Serial.print(" ("); Serial.print(vorigeMessung_B);Serial.println(")");
  diff_A = abs(analog_A - vorigeMessung_A);
  diff_B = abs(analog_B - vorigeMessung_B);
	if (diff_A < schwellwert_A) crossingIR_A = false; else crossingIR_A = true; // false setzt es auf jeden FAll zurueck
  if (diff_B < schwellwert_B) crossingIR_B = false; else crossingIR_B = true;
  vorigeMessung_A = analog_A;
  vorigeMessung_B = analog_B;
}

void showDisplay(int zeile, int spalte, String text) {
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
//	boolean buzzerOn = false;
	boolean buzzerHasPlayed = true;
	boolean loopActive = true;

	currentMillisLoop=millis();
//	if (intervalTone > 0) {
//		buzzerOn = true;
//		tone(buzzer,toneheigth);
//		//delay((int)intervalTone);
//		//noTone(buzzer);
//	}	
	
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
//			buzzerOn = false;
			//Serial.println("Ton aus: intervalTone abgelaufen");
		} 
			
		// IR auslesen und bei Durchfahrt (= Fehlstart) offset setzen bzw. Bahnstrom abschalten
		unsigned long currentMillisIr = millis();
		if (currentMillisIr - previousMillisIr >= intervalIr){
			previousMillisIr = currentMillisIr;
			readoutLanes();
			if (crossingIR_A) {
				if (versatz <= 6) versatz = 6; // linke rote LED an
					else versatz = 18; // beide Aussen LED  --> rot
				digitalWrite(rel_A, LOW); // Relais Bahn A abschalten
			} 	
			if (crossingIR_B) {
				if (versatz == 0) versatz = 12; // beide Aussen LED -->rot 
						else if (versatz == 6) versatz = 18;//rechte rote LED an
				digitalWrite(rel_B, LOW); // Relais Bahn A abschalten
			} 	
			Serial.println(versatz);	
			//Serial.println("IR auslesen");
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
	//int offset = 0; // Offset des Array Indesx im Fall eines  Fehlstarts. Fehlstart A: offset = 6, Fehlstart B: offset = 12 

	// Zeige Renninfo auf Display

	// Anzeige: Bereit?, Bestaetigen mit taste1
	// 5 Sekunden Countdown auf Display, Ton (buzzer)
	// s.a. https://electric-junkie.de/2020/08/arduino-millis-anstatt-von-delay/
	
	readoutLanes();
	// very first LED.
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
	
	i = 0;	
	while (i < 6) {
		int index = i + offset;
		// Serial.print(index); Serial.print("  "); Serial.println(Port_A[index],BIN);
		// Serial.print(index); Serial.print("  "); Serial.println(Port_B[index],BIN);
		writeRegister(MCP_GPIOA, Port_A[index]);
		writeRegister(MCP_GPIOB, Port_B[index]);
		startTone(tone_frequency[i],(unsigned long)tone_duration[i],offset,Port_A[index],Port_B[index]);
		Serial.println(offset);
		i++;
	};
}
 
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
 
  // put your setup code here, to run once:
  int sum1, sum2;
  for (int i = 1; i <= 20; i++) {
    sum1 = sum1 + analogRead(IR_A);
    sum2 = sum2 + analogRead(IR_B);
    delay(20);
  }
  vorigeMessung_A = sum1 / 20;
  vorigeMessung_B = sum2 / 20;
  //  Serial.println(vorigeMessung_A);
  //  Serial.println(vorigeMessung_B);
  besteZeit_A = 10000000.0;
  besteZeit_B = 10000000.0;
  
  Wire.begin();
  // alle Expander ports: OUTPUT
  writeRegister(MCP_IODIRA, 0b00000000);
  writeRegister(MCP_IODIRB, 0b00000000);
  
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
	// dummy readoutLanes():
	readoutLanes();
}


void raceLoop() {
  // aktives Rennen
	// diff_A B noch ersetetzen durch boolean
  analog_A = analogRead(IR_A);
  analog_B = analogRead(IR_B);
  // Debugging:
  //  Serial.print(analog_A); Serial.print(" ("); Serial.print(vorigeMessung_A);Serial.print(")");
  //  Serial.print("    ");
  //  Serial.print(analog_B); Serial.print(" ("); Serial.print(vorigeMessung_B);Serial.println(")");
  diff_A = abs(analog_A - vorigeMessung_A);
  diff_B = abs(analog_B - vorigeMessung_B);

  
  if (diff_A > schwellwert_A) {
    // AutoA durchfährt Lichtschranke
    Serial.print("Lichtschranke BahnA mit Runde: ");
    Serial.println(runde_A);

    if (n_A == IR_off_cycle) {
      myTime_A = millis();
      lapTime_A = float(myTime_A - startTime_A) / 1000;
      if (lapTime_A < besteZeit_A && runde_A > 0) {
        besteZeit_A = lapTime_A;
        besteRunde_A = runde_A;
      }
      if (runde_A > 0) {
//                Serial.println ("Auto 1 blau:");
//                Serial.print("letzte Runde: ");
//                Serial.print(runde_A);
//                Serial.print(" mit Zeit: ");
//                Serial.print(lapTime_A,3);
//                Serial.print("\t\t");
//                Serial.print("beste Runde: ");
//                Serial.print(besteRunde_A);
//                Serial.print(" mit Zeit: ");
//                Serial.println(besteZeit_A,3);
//                Serial.println();
//                Serial.println();
				String text;
				text = (String)(lapTime_A, 3);
				showDisplay(1,10,text);
				Serial.print("lapTime: ");Serial.print(text);
      }
      runde_A++;
      n_A--;
      //neue startTime festlegen:
      startTime_A = myTime_A;
    }
  }
  if (diff_B > schwellwert_B) {
    // Auto2 durchfährt Lichtschranke
    Serial.print("Lichtschranke BahnB mit Runde: ");
    Serial.println(runde_B);
      
    if (n_B == IR_off_cycle) {
      myTime_B = millis();
      lapTime_B = float(myTime_B - startTime_B) / 1000;
      if (lapTime_B < besteZeit_B && runde_B > 0) {
        besteZeit_B = lapTime_B;
        besteRunde_B = runde_B;
      }
      if (runde_B > 0) {
//                Serial.println ("Auto 2 gruen");
//                Serial.print("letzte Runde: ");
//                Serial.print(runde_B);
//                Serial.print(" mit Zeit: ");
//                Serial.print(lapTime_B,3);
//                Serial.print("\t\t");
//                Serial.print("beste Runde: ");
//                Serial.print(besteRunde_B);
//                Serial.print(" mit Zeit: ");
//                Serial.println(besteZeit_B,3);
//                Serial.println();
//                Serial.println();
        //anzeige(1, 'B', (String)runde_B, String(lapTime_B, 3), "");  <-- ersetzen durch showDisplay
      }
      runde_B++;
      n_B--;
      //neue startTime festlegen:
      startTime_B = myTime_B;
    }
  }

  // IR Brücke evtl. zurücksetzen und IR Messung für Vergleich speichern
  if (n_A < IR_off_cycle) n_A--;
  if (n_A == 0) n_A = IR_off_cycle;
  if (n_B < IR_off_cycle) n_B--;
  if (n_B == 0) n_B = IR_off_cycle;
  vorigeMessung_A = analog_A;
  vorigeMessung_B = analog_B;
  
  // IR Sensoren erneut auslesen.
  delay(IR_sensor_speed);
}

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
		}
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
		}
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
	}
}
 
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
			showDisplay(0,0,"Zeitrennen");
			showDisplay(0,11,(String)rennDauer);
			showDisplay(0,13,"Min.");
		} else {
			showDisplay(0,0,"Rundenrennen");
			showDisplay(0,13,(String)rundenAnzahl);
			showDisplay(0,16,"R.");
		}
		digitalWrite(rel_A, HIGH); 
		digitalWrite(rel_B, HIGH);
<<<<<<< Updated upstream
		//startingSignal(); 	
=======
		startingSignal(); 	
		String stringOne = "Hallo";
		showDisplay(3,0,stringOne);
		Serial.println(1.2345, 3);
		btn4.update();
		while (!btn4.rose()) raceLoop(); // noch aendern!
>>>>>>> Stashed changes
		// Anzeige auf Display
		readoutLanes();
		if (diff_A > diffMax_A) {
			diffMax_A = diff_A;
			Serial.print("diffMax_A: "); Serial.println(diffMax_A);
		};
		if (diff_B > diffMax_B) {
			diffMax_B = diff_B;
			diffMax_A = diff_B;
			Serial.print("diffMax_B: "); Serial.println(diffMax_B);
		};
		showDisplay(3,0,"                    ");
		showDisplay(0,0,"*** Testmode ***");
		showDisplay(1,0,(String)analog_A);
		showDisplay(2,0,(String)diff_A);
		showDisplay(3,0,(String)diffMax_A);
		showDisplay(1,10,(String)analog_B);
		showDisplay(2,10,(String)diff_B);
		showDisplay(3,10,(String)diffMax_B);
		btn4.update();
		if (btn4.rose()) {
			diffMax_A = 0;
			diffMax_B = 0;
		}	
		delay(IR_sensor_speed);
	}
}
