#ifndef SOLDER_STATION_H
#define SOLDER_STATION_H

#include "Arduino.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <EEPROM.h>
#include "max6675.h"
#include <Bounce2.h>

// SPI BUS:
#define SCK_PIN 13
#define SO_PIN 12
#define SOLDER_CS 11
#define DESOLDER_CS 10

// MAX6675 specific definitions
#define MAX6675_INIT_DELAY 1000LU
#define MAX6675_ACQ_INTERVAL 230LU
#define MAX6675_AVG_ALPHA 0.1f
#define MAX6675_MAX_TEMP 1000.f
#define MAX6675_MIN_TEMP 0.0f

// PID
#define PID_INTERVAL_F 250.f
#define PID_INTERVAL 250LU

typedef struct TOKEN{
    char index;
    char length;
};

enum CONTROL
{
    MANUAL = 0,
    AUTOMATIC = 1
};

CONTROL control = CONTROL::MANUAL;

enum CHANNEL
{
    IDLE = 0,
    SOLDER = 1,
    DESOLDER = 2
};

// Structures
typedef struct PID_settings
{
    // Kp -  proportional gain
    float Kp;
    // Ki -  integral gain
    float Ki;
    // Kd -  derivative gain
    float Kd;
    // max - maximum value of manipulated variable
    float max;
    // min - minimum value of manipulated variable
    float min;
    // hold point
    float hold_point;
    // Tau 
    float tau;
};

typedef struct IRON
{
    const char *name;
    char output_pin;
    char raw_output;
    char hold_flag;
    float power;
    float set_point;
    float temperature;
    float integral;
    float proportional;
    float derivative;
    float previous_set_point;
    float previous_error;
    float previous_temperature;
    unsigned long previous_read_time;
    unsigned long previous_pid_time;
    uint32_t eeprom_addr;
    PID_settings settings;
    MAX6675 tc;

    IRON(const char *_NAME, char _TC_PIN, char _OUT_PIN, uint32_t EEPROM_ADDR) : name{_NAME},
                                                                                 tc{SCK_PIN, _TC_PIN, SO_PIN},
                                                                                 output_pin{_OUT_PIN}
    {
        temperature = -100.0f;
        pinMode(_OUT_PIN, OUTPUT);
        analogWrite(_OUT_PIN, 0);
        eeprom_addr = EEPROM_ADDR;
        EEPROM.get(eeprom_addr, settings);
    };

    void save_eprom(void){
        Serial.print(F("Save current settings to EEPROM of "));
        Serial.println(name);
        EEPROM.put(eeprom_addr, settings);
    }

    void read_eeprom(void){
        Serial.print(F("Read current settings from EEPROM of "));
        Serial.println(name);
        EEPROM.get(eeprom_addr, settings);
    }

    void read_temp(void)
    {
        float value;
        unsigned long _tmp = millis();
        if ((_tmp - previous_read_time) >= MAX6675_ACQ_INTERVAL)
        {
            value = tc.readCelsius();
            if (temperature < 0.0f)
                temperature = value;
            temperature = constrain(
                    round((MAX6675_AVG_ALPHA * value) + (1.0f - MAX6675_AVG_ALPHA) * temperature),
                    MAX6675_MIN_TEMP,
                    MAX6675_MAX_TEMP);
            previous_read_time = _tmp;
        }
    }

    void write_output(void)
    {
        // Write output
        switch (control)
        {
        case CONTROL::MANUAL:
            analogWrite(output_pin, (uint8_t)(round(power)));
            break;
        case CONTROL::AUTOMATIC:
            analogWrite(output_pin, raw_output);
            break;
        default:
            analogWrite(output_pin, 0);
            break;
        }
    }
};

// LCD settings:
#define LCD_ADDR 0x3F
#define LCD_ROWS 4
#define LCD_COLS 20
// Screen refresh interval
#define LCD_FRAME_TIME 500
#define STR_BUFFER_SIZE 64
#define FLOAT_PRECISION 7
#define BUFFER_INDEX_INC(x) (x > (STR_BUFFER_SIZE - 1) ? (x=0) : x++)

// Hardware pins definitions:
// I2C BUS:
#define SDA_PIN A4
#define SCL_PIN A5

// Output to MOSFET gates:
#define SOLDER_PWM_PIN 9
#define DESOLDER_PWM_PIN 3
#define MAX_CELSIUS_OUTPUT 500.0f
#define MIN_CELSIUS_OUTPUT 0.0f

// User interface switches:
#define SW_COUNT 5
#define SW_BOUNCE_INTERVAL 15
#define SW_L_PIN 7
#define SW_R_PIN 5
#define SW_C_PIN 6
#define SW_UP_PIN 8
#define SW_DOWN_PIN A0
// Just for easier coding
#define SW_L 0
#define SW_C 1
#define SW_R 2
#define SW_UP 3
#define SW_DOWN 4
// Button hold settings
#define FIRST_STAGE_INTERVAL 1000LU
#define SECOND_STAGE_INTERVAL 3500LU
#define SW_REFRESH_INTERVAL 250LU
#define FIRST_STAGE_DELTA 1.0f
#define SECOND_STAGE_DELTA 10.0f
#define THIRD_STAGE_DETLA 25.0f

// Serial communication:
#define SERIAL_BAUD_RATE 9600LU
#define TOKEN_MAX 5
// Strings:
#define VALUE_SET F("New value set")
#define INVALID_COMMAND Serial.println(F("Invalid command"))

#define SOLDER_EEPROM_ADDR 0x0
#define DESOLDER_EEPROM_ADDR (sizeof(PID_settings))

/* Returns first position on LCD for cursors, to make
    string appear on center of the screen */
char lcd_center_string(char *string);
char lcd_center_string(const __FlashStringHelper *string);

void serial_process();
void serial_print_settings(IRON *obj);
void serial_print_pid(IRON *obj);
void serial_print_temperature(IRON *obj);


char match_str(const char *str_a, const char *str_b);
char match_str(const __FlashStringHelper *str_a, const TOKEN token);

void channel_idle(void);
void channel_solder(void);
void channel_desolder(void);

void pid_compute(IRON *obj);

#endif
