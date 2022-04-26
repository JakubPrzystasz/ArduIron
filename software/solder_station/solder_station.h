#ifndef SOLDER_STATION_H
#define SOLDER_STATION_H

#include "Arduino.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <EEPROM.h>
#include <max6675.h>
#include <Bounce2.h>

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
    // thermo factor
    float thermo_factor;
};

typedef struct IRON
{
    const __FlashStringHelper*name;
    unsigned long previous_time;
    char output_pin;
    char hold_state;
    float power;
    float set_point;
    float previous_set_point;
    float previous_error;
    float cumulative_error;
    float temperature;
    PID_settings settings;
};

// LCD custom characters:
char lcd_arrow_up[] = {0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x04, 0x00};

// LCD settings:
#define LCD_ADDR 0x3F
#define LCD_ROWS 4
#define LCD_COLS 20
// Screen refresh interval
#define LCD_FRAME_TIME 250
// Degree character
#define LCD_DEG_CHAR 223
#define STR_BUFFER_SIZE 64

// Hardware pins definitions:
// I2C BUS:
#define SDA_PIN A4
#define SCL_PIN A5

// SPI BUS:
#define SCK_PIN 13
#define SO_PIN 12
#define SOLDER_CS 11
#define DESOLDER_CS 10

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
#define SW_REFRESH_INTERVAL 200LU
#define FIRST_STAGE_DELTA 1.0f
#define SECOND_STAGE_DELTA 10.0f
#define THIRD_STAGE_DETLA 25.0f
 

// MAX6675 specific definitions
#define MAX6675_INIT_DELAY 500LU
#define MAX66575_ACQ_INTERVAL 1000LU
#define MAX66575_ACQ_INTERVAL_F 1000.0f

// Serial communication:
#define SERIAL_BAUD_RATE 115200LU

// Strings:
#define VALUE_SET F("New value set")

#define SOLDER_EEPROM_ADDR 0x0
#define DESOLDER_EEPROM_ADDR (sizeof(PID_settings))

/* Returns first position on LCD for cursors, to make
    string appear on center of the screen */
char lcd_center_string(char *string);
char lcd_center_string(const __FlashStringHelper *string);

// Setup IO for SPI slave
void init_spi_slave(char pin);

// Read temperature from MAX6675 slave
float read_temperature(char cs_pin);

void serial_print_settings(IRON *obj);
void serial_print_pid(IRON *obj);

char match_str(const char *str_a, const char *str_b);
char match_str(const __FlashStringHelper *str_a, const char *str_b);

void channel_idle(void);
void channel_solder(void);
void channel_desolder(void);


void pid_compute(IRON *obj);

#endif
