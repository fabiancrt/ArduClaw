#include <Wire.h>
#include <MPU6050.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <IRremote.h>
#include "DHT.h"
#include <math.h>   // For sqrt(), atan2()

// ------------------------------------------------------------------------
// Class for driving an LCD through a 74HC595 shift register
// ------------------------------------------------------------------------
class LiquidCrystalShift {
  public:
    // Constructor: set Arduino pins for the 74HC595
    LiquidCrystalShift(int dataPin, int clockPin, int latchPin);

    // Functions similar to the usual LiquidCrystal library:
    void begin(uint8_t cols, uint8_t rows);
    void clear();
    void home();
    void setCursor(uint8_t col, uint8_t row);
    void print(const char *str);
    void command(uint8_t cmd);
    void write(uint8_t data);

  private:
    int _dataPin, _clockPin, _latchPin;
    uint8_t _cols, _rows;

    // Lower-level helpers:
    void shiftOutByte(byte data);
    void pulseEnable(byte data);
    void sendNibble(byte nibble, bool rs);
    void sendByte(byte value, bool rs);
};

// Constructor
LiquidCrystalShift::LiquidCrystalShift(int dataPin, int clockPin, int latchPin)
  : _dataPin(dataPin), _clockPin(clockPin), _latchPin(latchPin) {}

// Sends a byte out to the shift register
void LiquidCrystalShift::shiftOutByte(byte data) {
  digitalWrite(_latchPin, LOW);
  shiftOut(_dataPin, _clockPin, MSBFIRST, data);
  digitalWrite(_latchPin, HIGH);
}

// Pulses the LCD's 'enable' pin through our shift register
void LiquidCrystalShift::pulseEnable(byte data) {
  shiftOutByte(data | 0x02);  // E high (bit 1)
  delayMicroseconds(2);
  shiftOutByte(data & ~0x02); // E low
  delayMicroseconds(100);
}

// Sends a 4-bit nibble (on bits 2-5) to the LCD
void LiquidCrystalShift::sendNibble(byte nibble, bool rs) {
  byte dataOut = (nibble & 0x0F) << 2; // shift nibble into bits 2-5
  if (rs) {
    dataOut |= 0x01; // Set RS if it's data
  }
  shiftOutByte(dataOut);
  pulseEnable(dataOut);
}

// Splits an 8-bit value into two nibbles and sends each
void LiquidCrystalShift::sendByte(byte value, bool rs) {
  sendNibble(value >> 4, rs);    // high nibble
  sendNibble(value & 0x0F, rs);  // low nibble
}

// Send a command (RS=0)
void LiquidCrystalShift::command(uint8_t cmd) {
  sendByte(cmd, false);
  delay(2);
}

// Send data (RS=1)
void LiquidCrystalShift::write(uint8_t data) {
  sendByte(data, true);
}

// Initializes the LCD for 4-bit mode via the 74HC595
void LiquidCrystalShift::begin(uint8_t cols, uint8_t rows) {
  _cols = cols;
  _rows = rows;

  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  pinMode(_latchPin, OUTPUT);

  delay(50); // Wait a bit for LCD to power up

  // "Waking" the LCD like standard docs suggest:
  sendNibble(0x03, false);
  delay(5);
  sendNibble(0x03, false);
  delay(5);
  sendNibble(0x03, false);
  delay(1);

  // Switch to 4-bit mode
  sendNibble(0x02, false);

  // Set function: 4-bit interface, 2 lines, 5x8 font
  command(0x28);

  // Turn display on, hide cursor, no blinking
  command(0x0C);

  // Entry mode: cursor moves right, no display shift
  command(0x06);

  clear();
}

// Clear the display
void LiquidCrystalShift::clear() {
  command(0x01);
  delay(2);
}

// Move cursor to top-left (home)
void LiquidCrystalShift::home() {
  command(0x02);
  delay(2);
}

// Position the cursor at (col, row)
void LiquidCrystalShift::setCursor(uint8_t col, uint8_t row) {
  const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row >= _rows) row = _rows - 1;
  command(0x80 | (col + row_offsets[row]));
}

// Print a string
void LiquidCrystalShift::print(const char *str) {
  while (*str) {
    write(*str++);
  }
}

// ------------------------------------------------------------------------
// PIN & SENSOR DEFINITIONS
// ------------------------------------------------------------------------
#define JOYSTICK_X_PIN          A2    // Joystick X-axis
#define JOYSTICK_SW_PIN         A1    // Joystick switch (toggle servo or stepper)
#define ULTRASONIC_TRIGGER_PIN  7     // Ultrasonic trigger
#define ULTRASONIC_ECHO_PIN     6     // Ultrasonic echo
#define PIR_PIN                 9     // PIR sensor pin
#define SERVO_PIN               5     // Servo control pin
#define WATER_SENSOR_PIN        10    // Water detection sensor
#define IR_RECEIVER_PIN         A0    // IR receiver
#define DHTPIN                  A3    // Temp/Humidity sensor pin

// Temperature & humidity thresholds
#define TEMP_THRESHOLD      50.0    
#define HUMID_THRESHOLD     90.0    

// Distance safety and "home" reference
#define EMERGENCY_DISTANCE_THRESHOLD 20.0  
#define HOME_POSITION_STEPS 0             

// IR Remote codes
#define IR_POWER_CODE     0xE318261B
#define IR_VOL_PLUS_CODE  0x511DBB
#define IR_VOL_MINUS_CODE 0xA3C8EDDB

// Stepper & calibration
#define CALIBRATION_POSITION 10         
#define CALIBRATION_TIMEOUT 3000        
#define MIN_BASE_SPEED 100              
#define MAX_BASE_SPEED 1000             
long baseSpeed = 300;                   
#define CORRECTION_FACTOR 0.1           
#define MAX_CORRECTION_SPEED 50         
#define JOYSTICK_CALIBRATION_DELAY 2000 

// Global variables for MPU6050
MPU6050 mpu;
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;
float pitch = 0.0;      
float safeAngle = 0.0;  
const float alpha = 0.98; 

// Timing intervals
unsigned long lastSensorUpdate = 0;
const unsigned long sensorInterval = 20; 
unsigned long lastUltrasonicUpdate = 0;
const unsigned long ultrasonicInterval = 100;
long lastDistance = 100;  

unsigned long lastJoystickMovementTime = 0;

// Servo details
Servo myServo;
int currentAngle = 90;
int targetAngle = 90;
const int servoStepSize = 1;  

// Debounce for joystick button
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool buttonHandled = false;
bool servoEnabled = false;  // false = stepper mode, true = servo mode

// PIR safety mode
bool safetyMode = false;
unsigned long pirStartTime = 0;

// LCD update timing
unsigned long lastLCDUpdate = 0;
const unsigned long lcdUpdateInterval = 500; 

// IR remote objects
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;
unsigned long lastIRCode = 0;  

// DHT sensor (using DHT11)
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Create our custom LCD object (Shifted)
LiquidCrystalShift lcd(2, 4, 3);

// Keeps track if system is on/off
bool systemActive = false;

// Stepper calibration flags
bool calibrating = false;
unsigned long calibratingStartTime = 0;

// Stepper setup (HALF4WIRE means half-stepping)
AccelStepper stepper(AccelStepper::HALF4WIRE, 8, 11, 12, 13);

// ------------------------------------------------------------------------
// FUNCTIONS
// ------------------------------------------------------------------------

// Calibrate MPU6050 with multiple samples
void calibrateSensor() {
  const int samples = 1000;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  Serial.println("Calibrating MPU6050... please hold still!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(2);
  }

  ax_offset = ax_sum / samples;
  ay_offset = ay_sum / samples;
  az_offset = az_sum / samples;
  gx_offset = gx_sum / samples;
  gy_offset = gy_sum / samples;
  gz_offset = gz_sum / samples;

  Serial.println("MPU6050 Calibration done!");
  delay(500);
  lcd.clear();
}

// Uses complementary filter to compute pitch angle
// Called every sensorInterval ms
void updateGyro() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Offset-correct the readings
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;

  // Convert to g's (assuming 16384 LSB/g on the accelerometer)
  float ax_g = (float)ax / 16384.0;
  float ay_g = (float)ay / 16384.0;
  float az_g = (float)az / 16384.0;

  // Pitch angle from accelerometer
  float pitchAcc = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

  // Gyro rate (assuming 131 LSB/°/s for default)
  float gyroRate = (float)gy / 131.0;

  // Calculate delta time
  float dt = sensorInterval / 1000.0;

  // Complementary filter
  pitch = alpha * (pitch + gyroRate * dt) + (1.0 - alpha) * pitchAcc;
}

// Returns ultrasonic distance in cm
long readUltrasonicDistance() {
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);

  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000);
  long distanceCm = duration / 58;
  return distanceCm;
}

// ------------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // Begin I2C and MPU6050 initialization
  Wire.begin();
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  delay(1000);
  calibrateSensor();

  // Fetch initial reading for safeAngle
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int16_t ax_corr = ax - ax_offset;
  int16_t ay_corr = ay - ay_offset;
  int16_t az_corr = az - (az_offset - 16384);
  float ax_g = (float)ax_corr / 16384.0;
  float ay_g = (float)ay_corr / 16384.0;
  float az_g = (float)az_corr / 16384.0;
  float pitchAcc = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
  safeAngle = pitchAcc;
  pitch = safeAngle;

  Serial.print("Initial Safe Angle = ");
  Serial.println(safeAngle);

  // Stepper config
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  stepper.setCurrentPosition(HOME_POSITION_STEPS);

  // Set up the servo
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  // Pin setups
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);
  pinMode(WATER_SENSOR_PIN, INPUT);

  // LCD initialization
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("System Ready");
  delay(1000);
  lcd.clear();

  // IR receiver
  irrecv.enableIRIn();

  // DHT
  dht.begin();

  Serial.println("Setup complete.");
}

// ------------------------------------------------------------------------
// LOOP
// ------------------------------------------------------------------------
void loop() {
  // Update pitch from MPU6050 at intervals
  if (millis() - lastSensorUpdate >= sensorInterval) {
    updateGyro();
    lastSensorUpdate = millis();
  }

  // Check environment with DHT sensor
  float currentTemp = dht.readTemperature();
  float currentHumid = dht.readHumidity();
  bool envShutdown = false;
  if (!isnan(currentTemp) && !isnan(currentHumid)) {
    if (currentTemp > TEMP_THRESHOLD || currentHumid > HUMID_THRESHOLD) {
      envShutdown = true;
    }
  }

  // IR remote handling
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;
    if (code == 0xFFFFFFFF) {
      code = lastIRCode;
    } else {
      lastIRCode = code;
    }

    // Power toggle
    if (code == IR_POWER_CODE) {
      if (!systemActive) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Loading...");
        delay(1500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Welcome!");
        delay(1000);
        lcd.clear();
        systemActive = true;
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Shutting Down");
        delay(1500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Power is Off");
        delay(1500);
        systemActive = false;
      }
    }
    // Adjust stepper speed if system is active
    else if (systemActive) {
      if (code == IR_VOL_PLUS_CODE) {
        baseSpeed += 50;
        if (baseSpeed > MAX_BASE_SPEED) {
          baseSpeed = MAX_BASE_SPEED;
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Stepper Speed +");
        lcd.setCursor(0, 1);
        char buf[17];
        snprintf(buf, sizeof(buf), "Speed: %ld", baseSpeed);
        lcd.print(buf);
        delay(1000);
        lcd.clear();
      } else if (code == IR_VOL_MINUS_CODE) {
        baseSpeed -= 50;
        if (baseSpeed < MIN_BASE_SPEED) {
          baseSpeed = MIN_BASE_SPEED;
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Stepper Speed -");
        lcd.setCursor(0, 1);
        char buf[17];
        snprintf(buf, sizeof(buf), "Speed: %ld", baseSpeed);
        lcd.print(buf);
        delay(1000);
        lcd.clear();
      }
    }
    irrecv.resume();
  }

  // If environment is too hot/humid, shut down
  if (envShutdown) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp/Hum Too High");
    lcd.setCursor(0, 1);
    lcd.print("System Shutdown");
    if (!servoEnabled) {
      stepper.moveTo(stepper.currentPosition());
    }
    delay(500);
    return;
  }

  // If system is off, just show a message and wait
  if (!systemActive) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Power is Off");
    lcd.setCursor(0, 1);
    lcd.print("Press Red Btn");
    delay(500);
    return;
  }

  // Water sensor override
  static bool waterOverrideActive = false;
  bool waterDetected = (digitalRead(WATER_SENSOR_PIN) == HIGH);
  if (waterDetected && !waterOverrideActive) {
    waterOverrideActive = true;
    Serial.println("Water detected! Initiating override...");

    if (servoEnabled) {
      int overrideAngle = 20; // move servo to reduce risk
      int newAngle = currentAngle - overrideAngle;
      if (newAngle < 0) newAngle = 0;
      myServo.write(newAngle);
      currentAngle = newAngle;
    } else {
      stepper.moveTo(stepper.currentPosition() - 300);
    }
    delay(1000);  

    // Reset to default
    if (servoEnabled) {
      currentAngle = 90;
      targetAngle = 90;
      myServo.write(90);
    } else {
      stepper.moveTo(CALIBRATION_POSITION);
    }
    return;
  } else if (!waterDetected) {
    waterOverrideActive = false;
  }

  // Safety checks (Ultrasonic + PIR)
  if (millis() - lastUltrasonicUpdate >= ultrasonicInterval) {
    lastDistance = readUltrasonicDistance();
    lastUltrasonicUpdate = millis();
  }

  bool emergencyTriggered = false;
  if (lastDistance > 0 && lastDistance < EMERGENCY_DISTANCE_THRESHOLD) {
    emergencyTriggered = true;
  }

  if (digitalRead(PIR_PIN) == HIGH) {
    if (pirStartTime == 0) {
      pirStartTime = millis();
    } else if (millis() - pirStartTime > 1000) {
      safetyMode = true;
    }
  } else {
    pirStartTime = 0;
    safetyMode = false;
  }

  if (emergencyTriggered || safetyMode) {
    stepper.moveTo(CALIBRATION_POSITION);
    servoEnabled = false;
  }

  // Joystick switch to toggle servo/stepper
  int toggleReading = digitalRead(JOYSTICK_SW_PIN);
  if (toggleReading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (toggleReading == LOW && !buttonHandled) {
      servoEnabled = !servoEnabled;
      buttonHandled = true;
      lastJoystickMovementTime = millis();
      lcd.clear();
      lcd.setCursor(0, 0);
      if (servoEnabled) {
        lcd.print("Servo Mode ON");
      } else {
        lcd.print("Stepper Mode ON");
      }
      delay(500);
      lcd.clear();
    }
    if (toggleReading == HIGH) {
      buttonHandled = false;
    }
  }
  lastButtonState = toggleReading;

  // Servo or Stepper control logic
  if (servoEnabled) {
    // Servo control
    int joyVal = analogRead(JOYSTICK_X_PIN);
    if (joyVal > 900 || joyVal < 100) {
      lastJoystickMovementTime = millis();
    }
    if (joyVal >= 100 && joyVal <= 900) {
      targetAngle = 90;
    } else if (joyVal > 900) {
      float ratio = (float)(joyVal - 900) / (1023 - 900);
      int angleOffset = (int)(ratio * 90);
      targetAngle = 90 + angleOffset;
    } else if (joyVal < 100) {
      float ratio = (float)(100 - joyVal) / 100;
      int angleOffset = (int)(ratio * 90);
      targetAngle = 90 - angleOffset;
    }

    if (currentAngle < targetAngle) {
      currentAngle += servoStepSize;
      if (currentAngle > targetAngle) currentAngle = targetAngle;
      myServo.write(currentAngle);
    } else if (currentAngle > targetAngle) {
      currentAngle -= servoStepSize;
      if (currentAngle < targetAngle) currentAngle = targetAngle;
      myServo.write(currentAngle);
    }
  } else {
    // Stepper control
    int joyVal = analogRead(JOYSTICK_X_PIN);
    if (joyVal > 900) {
      lastJoystickMovementTime = millis();
      calibrating = false;
      float ratio = (float)(joyVal - 900) / (1023 - 900);
      long speed = baseSpeed + (MAX_BASE_SPEED - baseSpeed) * ratio;
      stepper.setSpeed(speed);
      stepper.runSpeed();
    } else if (joyVal < 100) {
      lastJoystickMovementTime = millis();
      calibrating = false;
      float ratio = (float)(100 - joyVal) / 100;
      long speed = baseSpeed + (baseSpeed - MIN_BASE_SPEED) * ratio;
      stepper.setSpeed(-speed);
      stepper.runSpeed();
    } else {
      // Joystick is neutral
      if (millis() - lastJoystickMovementTime > JOYSTICK_CALIBRATION_DELAY) {
        if (!calibrating) {
          calibrating = true;
          calibratingStartTime = millis();
        }
      }
      if (calibrating) {
        if (millis() - calibratingStartTime < CALIBRATION_TIMEOUT) {
          long posError = CALIBRATION_POSITION - stepper.currentPosition();
          long correctionSpeed = (long)(posError * CORRECTION_FACTOR);
          if (correctionSpeed > MAX_CORRECTION_SPEED) correctionSpeed = MAX_CORRECTION_SPEED;
          if (correctionSpeed < -MAX_CORRECTION_SPEED) correctionSpeed = -MAX_CORRECTION_SPEED;
          stepper.setSpeed(correctionSpeed);
          stepper.runSpeed();
          lcd.setCursor(0, 0);
          lcd.print("Calibrating...");
        } else {
          stepper.moveTo(CALIBRATION_POSITION);
          stepper.run();
        }
      } else {
        stepper.moveTo(stepper.currentPosition());
        stepper.run();
      }
    }
  }

  // Update LCD every half second
  if (millis() - lastLCDUpdate >= lcdUpdateInterval) {
    lastLCDUpdate = millis();
    lcd.clear();

    if (digitalRead(WATER_SENSOR_PIN) == HIGH) {
      lcd.setCursor(0, 0);
      lcd.print("WATER OVERRIDE");
      lcd.setCursor(0, 1);
      lcd.print("Resetting...");
    }
    else if (emergencyTriggered || safetyMode) {
      lcd.setCursor(0, 0);
      lcd.print("EMERGENCY/Safety");
      lcd.setCursor(0, 1);
      char buf[17];
      snprintf(buf, sizeof(buf), "Dist: %ldcm", lastDistance);
      lcd.print(buf);
    }
    else if (servoEnabled) {
      lcd.setCursor(0, 0);
      lcd.print("Servo Mode");
      lcd.setCursor(0, 1);
      char buf[17];
      snprintf(buf, sizeof(buf), "Angle: %d", currentAngle);
      lcd.print(buf);
    }
    else {
      lcd.setCursor(0, 0);
      lcd.print("Stepper Mode");
      lcd.setCursor(0, 1);
      char buf[17];
      long currentPos = stepper.currentPosition();
      snprintf(buf, sizeof(buf), "Pos: %ld", currentPos);
      lcd.print(buf);
    }
  }

  delay(1); // Tiny delay for smooth operation
}