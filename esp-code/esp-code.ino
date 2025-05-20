#include <Wire.h>
#include <Arduino.h>
#include "rgb_lcd.h"

// === SWITCH PINS ===
#define POWER_SWITCH_PIN      5

// === POTENTIOMETER PINS ===
#define FAN_POT_PIN           34

// === LED PINS ===
#define POWER_LED_PIN         13

// === FAN PINS ===
#define FAN_PWM_PIN           25  // PWM control pin
#define FAN_TACHO_PIN         14  // RPM feedback pin

// === PWM CONFIG ===
#define PWM_FREQ              12500   // 12.5 kHz
#define PWM_CHANNEL           0
#define PWM_RESOLUTION        8       // 8-bit (0–255)

// === FAN SETTINGS ===
#define FAN_KICKSTART_SPEED   255     // Full speed
#define KICKSTART_DURATION    5000    // in milliseconds

// === I2C MULTIPLEXER ===
#define TCA_ADDR              0x70

// === GLOBAL VARIABLES ===
volatile unsigned long pulse_count = 0;
unsigned long last_time = 0;
unsigned long rpm = 0;

bool power_on = false;

int fan_speed = 178; // 70% default
int fan_speed_percentage = 0;

rgb_lcd lcd;
int fan_lcd_channel = 0;

// === FUNCTION DECLARATIONS ===
void tca_select(uint8_t channel);
void IRAM_ATTR count_pulse();
void calculate_fan_rpm();
void kickstart_fan();
void set_fan_speed();
void configure_lcd(int channel);
void reset_lcd(int channel);
int get_pot_value_percent(int pin);
int get_pot_value_8bit(int pin);
void display_lcd_generic(int channel, String header, double number, String zero_value);
void handle_power_off();
void control_fan();
void control_panel_controller(void *pv_parameters);
void switches_controller(void *pv_parameters);
void toggle_switch_generic(int switch_pin, bool &device_state, int led_pin, const char *device_name, void (*callback)() = nullptr, bool reverse_expected_value = false);
void scan_i2c_with_tca();


// === I2C Multiplexer Channel Select ===
void tca_select(uint8_t channel) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delay(10);
}

// === Tachometer Interrupt ===
void IRAM_ATTR count_pulse() {
    pulse_count++;
}

// === Calculate Fan RPM ===
void calculate_fan_rpm() {
    if (millis() - last_time >= 1000) {
        detachInterrupt(digitalPinToInterrupt(FAN_TACHO_PIN));
        rpm = (pulse_count * 30);  // 60s / 2 pulses = 30
        Serial.printf("Fan speed: %lu RPM\n", rpm);
        pulse_count = 0;
        last_time = millis();
        attachInterrupt(digitalPinToInterrupt(FAN_TACHO_PIN), count_pulse, FALLING);
    }
}

// === Kickstart Fan ===
void kickstart_fan() {
    Serial.println("Kickstarting fan...");
    ledcWrite(FAN_PWM_PIN, FAN_KICKSTART_SPEED);
    delay(KICKSTART_DURATION);
}

// === Set Fan Speed ===
void set_fan_speed() {
    ledcWrite(FAN_PWM_PIN, fan_speed);
}

// === Configure LCD ===
void configure_lcd(int channel) {
    tca_select(channel);
    delay(10);
    lcd.begin(16, 2);
    lcd.setRGB(255, 255, 0);  // Yellow backlight
    lcd.setCursor(0, 0);
    lcd.print("LCD 2 Ready");
}

// === Clear LCD ===
void reset_lcd(int channel) {
    tca_select(channel);
    delay(10);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
}

// === Read Potentiometer: % Output ===
int get_pot_value_percent(int pin) {
    uint16_t value = analogRead(pin);
    if (value <= 409) return 0; //Potentiometer below 10% can be formatted to 0%
    return map(value, 409, 4095, 0, 100);
}

// === Read Potentiometer: 8-bit Output ===
int get_pot_value_8bit(int pin) {
    uint16_t value = analogRead(pin);
    if (value <= 409) return 0; //Potentiometer below 10% can be formatted to 0%
    return map(value, 409, 4095, 26, 255);
}

// === Display on LCD ===
void display_lcd_generic(int channel, String header, double number, String zero_value) {
    tca_select(channel);
    delay(10);

    if (number > 0) {
        lcd.setCursor(0, 0);
        lcd.print(header);
        lcd.setCursor(0, 1);
        lcd.print(number);
        lcd.print("           ");
    } else {
        lcd.setCursor(0, 1);
        lcd.print(zero_value);
    }
}

// === Handle Power Off ===
void handle_power_off() {
    fan_speed = 0;
    reset_lcd(fan_lcd_channel);
    set_fan_speed();
}

// === Fan Control ===
void control_fan() {
    fan_speed_percentage = get_pot_value_percent(FAN_POT_PIN);
    fan_speed = get_pot_value_8bit(FAN_POT_PIN);
    display_lcd_generic(fan_lcd_channel, "Fan speed:", fan_speed_percentage, "Fan is off");
    set_fan_speed();
    calculate_fan_rpm();
}

// === Fan Panel Task (Core 0) ===
void control_panel_controller(void *pv_parameters) {
    configure_lcd(fan_lcd_channel);

    while (1) {
        if (!power_on) {
            handle_power_off();
            delay(500);
            continue;
        }
        control_fan();
        delay(100);
    }
}

// === Switches Task (Core 1) ===
void switches_controller(void *pv_parameters) {
    while (1) {
        toggle_switch_generic(POWER_SWITCH_PIN, power_on, POWER_LED_PIN, "Fan", nullptr, true);
        delay(100);
    }
}

// === Toggle Switch Handler ===
void toggle_switch_generic(int switch_pin, bool &device_state, int led_pin, const char *device_name, void (*callback)(), bool reverse_expected_value) {
    bool expected_value = reverse_expected_value ? HIGH : LOW;
    if (digitalRead(switch_pin) == expected_value) {
        device_state = !device_state;
        digitalWrite(led_pin, device_state);
        Serial.printf("%s is %s\n", device_name, device_state ? "ON" : "OFF");

        if (callback) {
            callback();
        }

        delay(1000);  // Debounce delay
    }
}

// === I2C Scanner for TCA ===
void scan_i2c_with_tca() {
    for (uint8_t ch = 0; ch < 8; ch++) {
        Serial.printf("Scanning channel %u\n", ch);
        tca_select(ch);
        delay(5);

        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                Serial.printf("  Device found at 0x%02X\n", addr);
            }
        }
    }
}

// === Arduino Setup ===
void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);
    pinMode(POWER_LED_PIN, OUTPUT);
    pinMode(FAN_TACHO_PIN, INPUT_PULLUP);

    ledcAttach(FAN_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
    attachInterrupt(digitalPinToInterrupt(FAN_TACHO_PIN), count_pulse, FALLING);

    xTaskCreatePinnedToCore(control_panel_controller, "controlPanelController", 10000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(switches_controller, "switchesController", 10000, NULL, 1, NULL, 1);

    kickstart_fan();
    last_time = millis();
}

// === Arduino Loop ===
void loop() {
    // Main logic is handled by tasks.
}
