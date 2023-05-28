#include <Wire.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

/******************ANPASSEN**************************************/



// varibale für maximale RPM
const unsigned int maxRPM = 3000;

// variable zum Tweeken des Multifunktionsschalter RPM Modus
const int tweekMult = 30;

//Faktor beim Halten
const float KVFaktor= 0.25f;
// keyvaluemin
const int kVMin = 128;

// offset des Inkrementalgebers
int OffsettInkr = 0;

// tolleranz für inkrementalposition
static const unsigned int itoll = 15;

LiquidCrystal_I2C lcd(0x78, 20, 4);  // set the LCD address to 0x78 for a 20 ints and 4 line display

/****************************************************************/

// Pins
static const unsigned int pInkremental = 0;
static const unsigned int pSwitchMult = 1;
static const unsigned int pLcdData = 2;
static const unsigned int PLcdClk = 3;
static const unsigned int D8 = 13;
static const unsigned int D7 = 5;
static const unsigned int D6 = 6;
static const unsigned int pHalleffekt = 7;
static const unsigned int D5 = 8;
static const unsigned int D4 = 9;
static const unsigned int D3 = 10;
static const unsigned int D2 = 11;
static const unsigned int D1 = 12;
static const unsigned int pInkrementalB = A3;
static const unsigned int pSwitchMultB = A2;
static const unsigned int pWorkmode = A1;
static const unsigned int pDir = A0;

// Variable for pos(0) and Turn (1) Mode
int workMode = 0;

// variable für die Drehrichtung CW(0) CCW(1)
int direction = 0;

// drehrichtung
const int dCW = 0;
const int dCCW = 1;

// Zähler für Multifunktionsswitch
int cSwitchMult = 0;
int cSwitchMultOld = 0;

// Zähler für Inkremental
int cInkremental = 0;

// timer für die Hallsensor RPM Messung
int oldTime = 0;

// timer für die mehrfachschaltung der RPM einstellung
int oldTimeTweekRPM = 0;

// gemessene RPM
float mRPM = 0.0f;

// gewünschte RPM
int dRPM = 0;

// wirkliche pos
int aPos = 0;

// gewünschte pos
int dPos = 0;

// delay für die RPM
unsigned long Delay = 0;

// Inkrementalwerte für die Positionen
static const unsigned int vPos1 = 0 * 128 + OffsettInkr;
static const unsigned int vPos2 = 1 * 128 + OffsettInkr;
static const unsigned int vPos3 = 2 * 128 + OffsettInkr;
static const unsigned int vPos4 = 3 * 128 + OffsettInkr;
static const unsigned int vPos5 = 4 * 128 + OffsettInkr;
static const unsigned int vPos6 = 5 * 128 + OffsettInkr;
static const unsigned int vPos7 = 6 * 128 + OffsettInkr;
static const unsigned int vPos8 = 7 * 128 + OffsettInkr;



/// @brief starts the lcd
void startLCD() {
  lcd.init();
  lcd.backlight();
  lcdStartUPMessage();
}

/// @brief prints the lcd picture for the startup mode
void lcdStartUPMessage() {
  noInterrupts();
  lcd.setCursor(2, 4);
  lcd.print("Hello there!");  //Gerneral Kenobi .... swwwwwwww (mutlipe lightsabers go on)
  lcd.setCursor(3, 4);
  lcd.print("Winter \& Rexroth");
  delay(1000);
  lcd.clear();
  for (int i = 0; i < 3; i++) {
    lcd.print("Loading.");
    delay(333);
    lcd.print("Loading..");
    delay(333);
    lcd.print("Loading...");
    delay(333);
    lcd.clear();
  }
  interrupts();
  workMode = digitalRead(pWorkmode);
  switch (workMode) {
    case 1:
      direction = digitalRead(pDir);
      lcdTurn();
      break;
    case 0:
      lcdStep();
    default:
      /* code */
      break;
  }
}

/// @brief prints the lcd picture for the turn mode
void lcdTurn() {
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Turn");
  lcd.setCursor(0, 17);
  lcd.print(direction);
  lcd.setCursor(1, 3);
  lcd.print("desired:");
  lcd.setCursor(1, 12);
  lcd.print(dRPM);
  lcd.setCursor(2, 4);
  lcd.print("actual:");
  lcd.setCursor(2, 12);
  lcd.print(mRPM);
  lcd.setCursor(3, 0);
  lcd.print("Info:");
  lcd.setCursor(3, 6);
  lcd.print("max rpm=");
  lcd.setCursor(3, 15);
  lcd.print(maxRPM);
}

/// @brief prints the lcd picture for the step mode
void lcdStep() {
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Step");
  lcd.setCursor(1, 3);
  lcd.print("desired:");
  lcd.setCursor(1, 12);
  lcd.print(dPos);
  lcd.setCursor(2, 4);
  lcd.print("actual:");
  lcd.setCursor(2, 12);
  lcd.print(aPos);
  lcd.setCursor(3, 0);
  lcd.print("Info:");
  lcd.setCursor(3, 6);
  lcd.print("encoder=");
  lcd.setCursor(3, 15);
  lcd.print(cInkremental);
}

/// @brief gets the PWM value for the N channel mosfets
/// @return
int getKValue() {
  int kValue = kVMin;

  kValue = map(mRPM, 0, maxRPM, kVMin, 255);
  return kValue;
}

/// @brief defines the Pinmodes
void pinModeDefinitions() {

  // OUTPUT Pins

  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);

  // Interrupt Pins
  pinMode(pHalleffekt, INPUT);   // Halleffekt
  pinMode(pInkremental, INPUT);  // Inkremental
  pinMode(pSwitchMult, INPUT);   // Switch_Mult

  // INPUT Pins
  pinMode(pInkrementalB, INPUT);  // Inkremantal_B
  pinMode(pSwitchMultB, INPUT);   // Switch_Mult_B
  pinMode(pWorkmode, INPUT);      // Switch_Mode
  pinMode(pDir, INPUT);           // Switch_Dir
}

/// @brief creates the Interrupt Routienes
void attachInterrupts() {

  attachInterrupt(digitalPinToInterrupt(pHalleffekt), Halleffekt, RISING);    // Halleffektsensor
  attachInterrupt(digitalPinToInterrupt(pInkremental), Inkremental, FALLING);  // Inkrementalgeber
  attachInterrupt(digitalPinToInterrupt(pSwitchMult), Switch_Mult, CHANGE);   // Switch Mult
}

/// @brief pos0
/// @param kValue
void pos0(int kValue) {
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  analogWrite(D3, LOW);
  analogWrite(D4, LOW);
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  analogWrite(D7, LOW);
  analogWrite(D8, LOW);
}

/// @brief position1
/// @param kValue
void pos1(int kValue) {
  digitalWrite(D1, HIGH);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  analogWrite(D4, kValue);
  digitalWrite(D5, HIGH);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  analogWrite(D8, kValue);
}

/// @brief position2
/// @param kValue
void pos2(int kValue) {
  digitalWrite(D1, HIGH);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  analogWrite(D4, kValue);
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);
}

/// @brief position3
/// @param kValue
void pos3(int kValue) {
  digitalWrite(D1, HIGH);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  analogWrite(D4, kValue);
  digitalWrite(D5, LOW);
  digitalWrite(D6, HIGH);
  analogWrite(D7, kValue);
  digitalWrite(D8, LOW);
}

/// @brief postition4
/// @param kValue
void pos4(int kValue) {
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  digitalWrite(D4, LOW);
  digitalWrite(D5, LOW);
  digitalWrite(D6, HIGH);
  analogWrite(D7, kValue);
  digitalWrite(D8, LOW);
}

/// @brief position5
/// @param kValue
void pos5(int kValue) {
  // Pos 5
  digitalWrite(D1, LOW);
  digitalWrite(D2, HIGH);
  analogWrite(D3, kValue);
  digitalWrite(D4, LOW);
  digitalWrite(D5, LOW);
  digitalWrite(D6, HIGH);
  analogWrite(D7, kValue);
  digitalWrite(D8, LOW);
}

/// @brief position6
/// @param kValue
void pos6(int kValue) {
  digitalWrite(D1, LOW);
  digitalWrite(D2, HIGH);
  analogWrite(D3, kValue);
  digitalWrite(D4, LOW);
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);
}

/// @brief position7
/// @param kValue
void pos7(int kValue) {
  digitalWrite(D1, LOW);
  digitalWrite(D2, HIGH);
  analogWrite(D3, kValue);
  digitalWrite(D4, LOW);
  digitalWrite(D5, HIGH);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  analogWrite(D8, kValue);
}

/// @brief postion8
/// @param kValue
void pos8(int kValue) {

  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  digitalWrite(D4, LOW);
  digitalWrite(D5, HIGH);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  analogWrite(D8, kValue);
}

/// @brief array of functionpointers for of the Positions
void (*Positions[])(int) = { pos0, pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8 };

/// @brief calls RPM Funktion when a full turn is done
void Halleffekt() {

  readmRPM();
  cInkremental = 0;
}

/// @brief calculates measurad (Hallsensor) RPM
int readmRPM() {
  mRPM = (float)60 / (1000 * (millis() - oldTime));
  oldTime = millis();
}

/// @brief hanges Counter given to the turn direction
void Inkremental() {
  if (digitalRead(pInkremental) != digitalRead(pInkrementalB)) {
    cInkremental++;
  } else {
    cInkremental--;
  }

  if (cInkremental < 1) {
    cInkremental = 1024;
  }
  if (cInkremental > 1024) {
    cInkremental = 1;
  }
}

/// @brief changes Counter given to the turn direction
void Switch_Mult() {
  if (digitalRead(pSwitchMult) != digitalRead(pSwitchMultB)) {
    cSwitchMult++;
    if (cSwitchMult > 24) {
      cSwitchMult = 1;
      cSwitchMultOld = 0;
    }
  } else {
    cSwitchMult--;
    if (cSwitchMult < 1) {
      cSwitchMult = 24;
      cSwitchMultOld = 25;
    }
  }
}

/// @brief calls the functions for the Turn mode
void Turn() {
  Delay = getDelay();
  if (Delay > 1) {
    if (direction == dCCW) {
      turnCCW(Delay);
    } else if (direction == dCW) {
      turnCW(Delay);
    }

    else {
      Positions[0](getKValue());
    }
  }
  if (Delay < 1) {
    Positions[0](kVMin);
      delay(1000);
  }
}

/// @brief turns the Motor cw with a desired RPM
/// @param delay
void turnCW(int iDelay) {
  int i = 1;
  while (i < 9) {
    Positions[i](getKValue());
    delay((int)iDelay);
    i++;
  }
}

/// @brief turns the Motor ccw with a desired RPM
/// @param delay
void turnCCW(int iDelay) {
  for (int i = 8; i--; i > 0) {
    Positions[i](getKValue());
    delay((int)iDelay);
  }
}

/// @brief calculates Delay for the Turnfunctions
/// @return needed delay (INT)
long getDelay() {
  createdRPM();
  if (dRPM == 0) {
    return 0;
  }
  return (long)(1000.0f) / ((dRPM / 60.0f) * 8.0f);
}

/// @brief creates desired RPM (switchMult)
void createdRPM() {
  if (cSwitchMult > cSwitchMultOld)
    dRPM += 50;
  else if (cSwitchMult < cSwitchMultOld) {
    dRPM -= 50;
  }
  
  if (dRPM < 0) {
    dRPM = 0;
  }

  if (dRPM > maxRPM) {
    dRPM = maxRPM;
  }

  cSwitchMultOld = cSwitchMult;
  oldTimeTweekRPM = millis();
}

/// @brief calls the functions for the step mode
void Step() {
  getDesiredPos();
  stepToPos(dPos);
  readAPos();

  if (dPos == 0) {
    Positions[0](kVMin);
  }
}

/// @brief gets the desired Position
void getDesiredPos() {
  if (cSwitchMult > cSwitchMultOld) {
    dPos++;
    if (dPos > 8) {
      dPos = 1;
    }
  } else if (cSwitchMult < cSwitchMultOld) {
    dPos--;
    if (dPos < 1) {
      dPos = 8;
    }
  }
  cSwitchMultOld = cSwitchMult;
}

/// @brief calculates the actual Position
void readAPos() {
  if (vPos1 - itoll > cInkremental || cInkremental < vPos1 + itoll) {
    aPos = 1;
  } else if (vPos2 - itoll < cInkremental < vPos2 + itoll) {
    aPos = 2;
  } else if (vPos3 - itoll < cInkremental < vPos3 + itoll) {
    aPos = 3;
  } else if (vPos4 - itoll < cInkremental < vPos4 + itoll) {
    aPos = 4;
  } else if (vPos5 - itoll < cInkremental < vPos5 + itoll) {
    aPos = 5;
  } else if (vPos6 - itoll < cInkremental < vPos6 + itoll) {
    aPos = 6;
  } else if (vPos7 - itoll < cInkremental < vPos7 + itoll) {
    aPos = 7;
  } else if (vPos8 - itoll < cInkremental < vPos8 + itoll) {
    aPos = 8;
  } else {
    aPos = 0;
  }
}

/// @brief steps to the desired Position
/// @param dPos
void stepToPos(int dPos) {
  if(dPos != aPos){
  Positions[dPos](kVMin);
  }
  else{
   Positions[dPos](KVFaktor*kVMin); 
  }
  
}

void setup() {

  // initialize the lcd
  //void startLCD();
  Serial.begin(9600);

  // startpositon
  Positions[1](kVMin);
  delay(1000);
  Positions[0](kVMin);

  // definitionen
  pinModeDefinitions();
  attachInterrupts();
}

void loop() {


  workMode = digitalRead(pWorkmode);
  direction = digitalRead(pDir);

  switch (workMode) {
    case 1:  //workmode = turn
      Turn();
      //lcdTurn();
      break;
    case 0:  //workmode = step
      Step();
      //lcdStep();
    default:
      //errorhandling
      break;
  }
  Serial.println(dRPM);
}