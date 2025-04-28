#include <Wire.h>;
#include "rgb_lcd.h"
#include <Arduino.h>;

//SWITCHES PINS
#define POWER_SWITCH_PIN 5
#define SPEAKER_POWER_SWITCH_PIN 4

//POT PINS
#define FAN_POT_PIN 34
#define VOLUME_POT_PIN 35

//LED PINS
#define POWER_LED_PIN 13
#define SPEAKER_POWER_LED_PIN 12

//FAN PINS
#define FAN_PWM_PIN 25    // PWM control pin
#define FAN_TACHO_PIN 14  // RPM feedback pin

// PWM properties
#define PWM_FREQ 25000    // 25kHz PWM frequency
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255)

// Fan speed settings
#define FAN_KICKSTART_SPEED 255  // 100% for kickstart
#define KICKSTART_DURATION 5000  // Kickstart duration in milliseconds

// Variables for RPM calculation
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
unsigned long rpm = 0;

bool POWER_ON = false;
bool SPEAKER_ON = false;

int FAN_SPEED = 178;     // 0-255, 70% default
int FAN_SPEED_PERCENTAGE = 0;
float SPEAKER_VOLUME = 0;


rgb_lcd lcd;

int fan_lcd_channel = 0;
int volume_lcd_channel = 1;

#define TCA_ADDR 0x70 

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10); 
}

// Interrupt function for tachometer
void IRAM_ATTR countPulse() {
  pulseCount++;
}

void calculateFanRpm() { 
  // Calculate and display RPM every second
  if (millis() - lastTime >= 1000) {
    // Disable interrupt temporarily to avoid race condition
    detachInterrupt(digitalPinToInterrupt(FAN_TACHO_PIN));
    
    // Calculate RPM (typically 2 pulses per revolution for ebm-papst fans)
    // Formula: (pulseCount * 60) / (time_in_seconds * pulses_per_revolution)
    rpm = (pulseCount * 30); // 60 seconds / 2 pulses per revolution = 30
    
    Serial.print("Fan speed: ");
    Serial.print(rpm);
    Serial.println(" RPM");
    
    // Reset counter and timer
    pulseCount = 0;
    lastTime = millis();
    
    // Re-enable interrupt
    attachInterrupt(digitalPinToInterrupt(FAN_TACHO_PIN), countPulse, FALLING);
  }
}

void kickstartFan() {
  Serial.println("Kickstarting fan...");
  ledcWrite(FAN_PWM_PIN, FAN_KICKSTART_SPEED);
  delay(KICKSTART_DURATION);
}

void setFanSpeed() {
  ledcWrite(FAN_PWM_PIN, FAN_SPEED);
}

void configureLcd(int channel) {
    tcaSelect(channel);
    delay(10);

    lcd.begin(16, 2);
    lcd.setRGB(255, 255, 0);  // Yellow backlight

    lcd.setCursor(0, 0);
    lcd.print("LCD 2 Ready");
}

void resetLcd(int channel) {
  tcaSelect(channel);
  delay(10);
  
  lcd.setCursor(0, 0);
  lcd.print("                ");

  lcd.setCursor(0, 1);
  lcd.print("                ");
}

int getPotentiometerValueInPercent(int pin) {
    return map(analogRead(pin), 0, 4095, 0, 100);
}

int getPotentiometerValueIn8Bit(int pin) {
    return map(analogRead(pin), 0, 4095, 0, 255);
}

void displayLcdGeneric(int channel, String header, double number, String zeroValue) {
    tcaSelect(channel);
    delay(10);

    if(number > 0){
      lcd.setCursor(0, 0);
      lcd.print(header);

      lcd.setCursor(0, 1); // Move to second row
      lcd.print(number);
      lcd.print("           "); 
    } else {
      lcd.setCursor(0, 1); // Move to second row
      lcd.print(zeroValue);
    }
}

void handlePowerOff() {
    // As long as the panel is off reset the values and keep waiting.
    FAN_SPEED = 0;
    resetLcd(fan_lcd_channel);
    setFanSpeed(); 

    if(!SPEAKER_ON) {
      SPEAKER_VOLUME = 0;
      resetLcd(volume_lcd_channel);
    }
}

void controlFan() {
    FAN_SPEED_PERCENTAGE = getPotentiometerValueInPercent(FAN_POT_PIN);
    FAN_SPEED = getPotentiometerValueIn8Bit(FAN_POT_PIN);

    displayLcdGeneric(fan_lcd_channel, "Fan speed:", FAN_SPEED, "Fan is off");
    setFanSpeed();
    calculateFanRpm();
}

void controlSpeaker() {
    SPEAKER_VOLUME = getPotentiometerValueInPercent(VOLUME_POT_PIN);
    displayLcdGeneric(volume_lcd_channel, "Speaker volume:", SPEAKER_VOLUME, "Speaker is off");
}

void controlPanelController(void *pvParameters) {
    configureLcd(fan_lcd_channel);
    configureLcd(volume_lcd_channel);

    while (1) {
      if(!POWER_ON) {
        handlePowerOff();
        delay(500);
        continue;
      }

      controlFan();

      if(!SPEAKER_ON) {
      //  controlSpeaker();      
      }

    delay(100);
  }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    scanI2CWithTCA();

    pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
    //pinMode(SPEAKER_POWER_SWITCH_PIN, INPUT_PULLUP);  // Use internal pull-up resistor

    pinMode(POWER_LED_PIN, OUTPUT);
    //pinMode(SPEAKER_POWER_LED_PIN, OUTPUT);

    ledcAttach(FAN_PWM_PIN, PWM_FREQ, PWM_RESOLUTION); // Set up PWM 
    pinMode(FAN_TACHO_PIN, INPUT_PULLUP); // Set up tacho meters pins
    attachInterrupt(digitalPinToInterrupt(FAN_TACHO_PIN), countPulse, FALLING);
    
    xTaskCreatePinnedToCore(controlPanelController, "controlPanelController", 10000, NULL, 1, NULL, 0); //Runs on core 0;
    xTaskCreatePinnedToCore(switchesController, "switchesController", 10000, NULL, 1, NULL, 1); //Runs on core 1;

    kickstartFan();
    lastTime = millis();
}

void powerDownReset() {
  if(!POWER_ON) {
    float SPEAKER_VOLUME = 0;
    digitalWrite(SPEAKER_POWER_LED_PIN, LOW); 
  }
}

void scanI2CWithTCA() {
  for (uint8_t ch = 0; ch < 8; ch++) {
    Serial.print("Scanning channel ");
    Serial.println(ch);
    tcaSelect(ch);
    delay(5);  // Give I2C path time to settle

    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("  Device found at 0x");
        Serial.println(addr, HEX);
      }
    }
  }
}

void toggleSwitchGeneric(int switchPin, bool &deviceState, int ledPin, const char *deviceName, void (*callback)() = nullptr) {

    if (digitalRead(switchPin) == LOW) {
        deviceState = !deviceState; // Toggle state
        digitalWrite(ledPin, deviceState); // Set LED accordingly
        Serial.print(deviceName);
        Serial.println(deviceState ? " is ON" : " is OFF");
        
        if (callback) {
          callback();
        }

        delay(1000); // Debounce delay
    }
}

// Runs on core 1;
void switchesController(void *pvParameters) {

  while(1) {
    toggleSwitchGeneric(POWER_SWITCH_PIN, POWER_ON, POWER_LED_PIN, "Fan", powerDownReset);
    if(POWER_ON) {
      //toggleSwitchGeneric(SPEAKER_POWER_SWITCH_PIN, SPEAKER_ON, SPEAKER_POWER_LED_PIN, "Speaker");
    }
    
    delay(100);
  }
}

void loop() { 

}

