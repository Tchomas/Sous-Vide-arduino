// Sous Vide + (ESP8266 not in use)
#include <Arduino.h>
#include <stdlib.h>
#include <PID_v1.h> // PID library
#include <LiquidCrystal.h>	// LCD
#include <OneWire.h>  // DS18B20 Temperature Sensor
#include <DallasTemperature.h>  // DS18B20 Temperature Sensor
#include <EEPROM.h> // Save and retrieve settings


// ************************************************
// Pin definitions
// ************************************************
#define RelayPin      13 // Output Relay
#define ONE_WIRE_BUS  4 // One-Wire Temperature Sensor

// Rotary encoder
#define PinCLK A1     //clk, A
#define PinDT  A2     //dt, B
#define PinSW  A3 // Push button switch
int pinALast = digitalRead(PinCLK);
int aVal;

// PID Variables
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid parameters
unsigned int Kp;
double Ki;
double Kd;

// EEPROM address
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000;
unsigned long windowStartTime;

const int logInterval = 5000; // log every x milliseconds
long lastLogTime = 0;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// LCD(rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(6, 7, 8, 9, 10, 11);

uint8_t smiley[8] = // define symbol
{
  B00000,
  B10001,
  B00000,
  B00000,
  B10001,
  B01110,
  B00000,
};
uint8_t degree[8] = // define symbol
{
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};

// ************************************************
// Setup
// ************************************************
void setup()
{  
  //Setpoint = 60;
  //Kp = 10000;
  //Ki = 0.2;
  //Kd = 0;
  //SaveParameters(); // Uncomment these once to manually set tuning parameters and/or setpoint

  // enable debug serial
  Serial.begin(9600);

  // Initialize Relay Control:
  pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(RelayPin, LOW);  // make sure it is off to start

  // LCD
  lcd.createChar(1, degree);  // Custom characters for lcd
  lcd.createChar(2, smiley);
  lcd.begin(16, 2);

  // to be moved...
  lcd.print("Startar...");
  Serial.println("Starting...");
  lcd.setCursor(14, 0);
  lcd.write(2);


  // Start up the DS18B20 One Wire Temperature Sensor
  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0))
  {
    while (true)
    {
      SensorError();
    }
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);

  delay(500);  // Splash screen


  // Initialize the PID and related variables
  LoadParameters();
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);

  // **********************************************
  // Start Sequence
  // **********************************************
  lcd.clear();
  Serial.println("Waiting for StartButton");
  while (digitalRead(PinSW))
  {
    SetTemp(1);
  }

  // **********************************************
  // Timer interrupt 100ms
  // **********************************************
  TCCR1B = 0b00011011;
  TCCR1A = 0b00000000;
  TIMSK1 = 0b00100000;
  ICR1 = 12499;       //8mHz clock -> ~100ms
  // Beräknad 10(ggr per sekund) = (8M(klockfrekvens)) / (2*32(prescaler arduino=32?)*(1+ICR1)). :ICR1=(ICR1H + ICR1L)
}

// Timer Interrupt Handler
SIGNAL(TIMER1_CAPT_vect)
{
  if ((Input < 1 || Input > 90) && millis() > 10000) // FAILSAFE
  {
    digitalWrite(RelayPin, LOW);
  }
  else DriveOutput();
}

// ************************************************
// Main Loop
// ************************************************
void loop() {

  // Prepare to transition to the RUN state
  sensors.requestTemperatures(); // Start an asynchronous temperature reading

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  windowStartTime = millis();

  Run();
}

void Run()
{
  SaveParameters();
  myPID.SetTunings(Kp, Ki, Kd);

  while (true)
  {
    // to be moved...
    lcd.clear();
    lcd.print("Nu: ");
    lcd.print(Input);
    lcd.write(1);
    lcd.print(" H:m");
    lcd.setCursor(0, 1);
    lcd.print("Sp: ");
    lcd.print(Setpoint);
    lcd.write(1);
    lcd.print(" ");
    lcd.print(millis() / 3600000);
    lcd.print(":");
    lcd.print((millis() / 60000) % 60);

    DoControl();

    // to be moved...
    // periodically log to serial port in csv format
    if (millis() - lastLogTime > logInterval)
    {
      SaveParameters();
      Serial.print(Input);
      Serial.print(",");
      Serial.print(Setpoint);
      Serial.print(",");
      Serial.println(Output);

      lastLogTime = millis();
    }
    delay(100);

    while (!digitalRead(PinSW))
    {
      SetTemp(0.1);
    }
  }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }

  // Execute control algorithm
  myPID.Compute();

  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output;
}


// ************************************************
// Called by timer interrupt (isr) to drive the output
// ************************************************
void DriveOutput()
{
  long now = millis();
  // "on time" is proportional to the PID output
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if ((onTime > 100) && (onTime > (now - windowStartTime)))
  {
    digitalWrite(RelayPin, HIGH);
  }
  else
  {
    digitalWrite(RelayPin, LOW);
  }
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
  if (Setpoint != EEPROM_readDouble(SpAddress))
  {
    EEPROM_writeDouble(SpAddress, Setpoint);
    Serial.print("Setpoint saved, ");
    Serial.println(Setpoint);
  }
  if (Kp != EEPROM_readDouble(KpAddress))
  {
    EEPROM_writeDouble(KpAddress, Kp);
    Serial.print("Kp saved, ");
    Serial.println(Kp);
  }
  if (Ki != EEPROM_readDouble(KiAddress))
  {
    EEPROM_writeDouble(KiAddress, Ki);
    Serial.print("Ki saved, ");
    Serial.println(Ki);
  }
  if (Kd != EEPROM_readDouble(KdAddress))
  {
    EEPROM_writeDouble(KdAddress, Kd);
    Serial.print("Kd saved, ");
    Serial.println(Kd);
  }

}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
  Setpoint = EEPROM_readDouble(SpAddress);
  Kp = EEPROM_readDouble(KpAddress);
  Ki = EEPROM_readDouble(KiAddress);
  Kd = EEPROM_readDouble(KdAddress);

  // Use defaults if EEPROM values are invalid
  if (isnan(Setpoint))
  {
    Setpoint = 62;
  }
  if (isnan(Kp))
  {
    Kp = 10000;
  }
  if (isnan(Ki))
  {
    Ki = 0.2;
  }
  if (isnan(Kd))
  {
    Kd = 0.0;
  }
}

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(address++, *p++);
  }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    *p++ = EEPROM.read(address++);
  }
  return value;
}


// ************************************************
// Error Handler
// ************************************************
void SensorError()
{
  digitalWrite(RelayPin, LOW);
  Serial.println(Input);
  Serial.println("Sensor error!");
  lcd.setCursor(0, 0);
  lcd.print("Sensor error!");
  delay(500);
  lcd.clear();
  delay(500);
  lcd.print("Sensor error!");
}

// ************************************************
// Change Setpoint with rotary encoder
// ************************************************
void SetTemp(float Multiple)
{
  int tt = 0;
  if (millis() - tt > 100 )
  {
    lcd.setCursor(0, 0);
    lcd.print("Set Temp: ");
    lcd.print(Setpoint);
    lcd.setCursor(0, 1);
    lcd.print("P");
    lcd.print(Kp);
    lcd.print(" I");
    lcd.print(Ki);

    tt = millis();
  }

  aVal = digitalRead(PinCLK);
  if (aVal != pinALast) // knob is rotating
  {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();
    // If interrupts come faster than 5ms, assume it's a bounce and ignore
    if (interruptTime - lastInterruptTime > 40) {
      if (digitalRead(PinDT) != aVal)  // A Changed first - Rotating Clockwise
      {
        Setpoint += (1 * Multiple);
      } else // Otherwise B changed first
      {
        Setpoint -= (1 * Multiple);
      }
    }
    pinALast = aVal;
    lastInterruptTime = interruptTime;
  }
}







