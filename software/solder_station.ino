#include "solder_station.h"

LiquidCrystal_I2C LCD(LCD_ADDR, LCD_COLS, LCD_ROWS);

IRON solder, desolder;
// SWITCHES DEBOUNCE
Bounce SWITCH[SW_COUNT];
char serial_buffer[STR_BUFFER_SIZE], serial_input_char, serial_tmp_index = -1;
char lcd_buffer[STR_BUFFER_SIZE], lcd_refresh = 1, channel, prev_channel;
unsigned long last_frame = 0, last_temp_acquisition = 0, last_button = 0;

void setup(void)
{
    // Initialize IO for hardware:
    pinMode(SOLDER_PWM_PIN, OUTPUT);
    pinMode(DESOLDER_PWM_PIN, OUTPUT);
    analogWrite(SOLDER_PWM_PIN, 0);
    analogWrite(DESOLDER_PWM_PIN, 0);

    pinMode(SW_L_PIN, INPUT_PULLUP);
    pinMode(SW_R_PIN, INPUT_PULLUP);
    pinMode(SW_C_PIN, INPUT_PULLUP);
    pinMode(SW_UP_PIN, INPUT_PULLUP);
    pinMode(SW_DOWN_PIN, INPUT_PULLUP);

    for (char i = 0; i < SW_COUNT; i++)
    {
        SWITCH[i] = Bounce();
        SWITCH[i].interval(SW_BOUNCE_INTERVAL);
    }
    SWITCH[SW_L].attach(SW_L_PIN, INPUT_PULLUP);
    SWITCH[SW_C].attach(SW_C_PIN, INPUT_PULLUP);
    SWITCH[SW_R].attach(SW_R_PIN, INPUT_PULLUP);
    SWITCH[SW_UP].attach(SW_UP_PIN, INPUT_PULLUP);
    SWITCH[SW_DOWN].attach(SW_DOWN_PIN, INPUT_PULLUP);

    solder.name = F("SOLDER");
    desolder.name = F("DESOLDER");

    solder.output_pin = SOLDER_PWM_PIN;
    desolder.output_pin = DESOLDER_PWM_PIN;

    // Initialize Serial communication:
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println(F("Initializing..."));
    Serial.println(F("To show help type: help"));

    // Initialize LCD:
    LCD.init();
    LCD.backlight();
    LCD.noCursor();
    LCD.home();
    LCD.createChar(0, (uint8_t *)lcd_arrow_up);
    LCD.setCursor(lcd_center_string(F("Initializing...")), 1);
    LCD.print(F("Initializing..."));

    // Init SPI BUS:
    pinMode(SO_PIN, INPUT);
    pinMode(SCK_PIN, OUTPUT);

    // Initialize SPI slaves:
    init_spi_slave(SOLDER_CS);
    init_spi_slave(DESOLDER_CS);

    // Read settings from EEPROM
    EEPROM.get(SOLDER_EEPROM_ADDR, solder.settings);
    EEPROM.get(DESOLDER_EEPROM_ADDR, desolder.settings);

    // MAX6675 chip needs some time to stabilize:
    delay(MAX6675_INIT_DELAY);
    LCD.clear();
}

void loop(void)
{
    // Process serial communication commands
    if (Serial.available())
    {
        serial_input_char = Serial.read();
        if (serial_input_char == '\r')
        {
            serial_buffer[++serial_tmp_index] = '\0';
            if (!strcmp_P(serial_buffer, (const char *)F("help")))
            {
                Serial.println(F("Help:"));
                Serial.println(F("To show current configuration:"));
                Serial.println(F("show [solder,desolder]"));
                Serial.println(F("To show current PID parameters:"));
                Serial.println(F("show [solder,desolder] pid"));
                Serial.println(F("To save/read to/from EEPROM:"));
                Serial.println(F("[save,read] [solder,desolder]"));
                Serial.println(F("To set new parameters for object:"));
                Serial.println(F("set [solder,desolder] [setpoint,holdpoint,min,max,thermo,Kp,Kd,Ki] value"));
            }
            else if (!strcmp_P(serial_buffer, (const char *)F("show solder")))
            {
                Serial.println(F("Current settings of SOLDER:"));
                serial_print_settings(&solder);
            }
            else if (!strcmp_P(serial_buffer, (const char *)F("show desolder")))
            {
                Serial.println(F("Current settings of DESOLDER:"));
                serial_print_settings(&desolder);
            }
            else if (!strcmp_P(serial_buffer, (const char *)F("save solder")))
            {
                Serial.println(F("Save current SOLDER settings to EEPROM"));
                EEPROM.put(SOLDER_EEPROM_ADDR, solder.settings);
            }
            else if (!strcmp_P(serial_buffer, (const char *)F("save desolder")))
            {
                Serial.println(F("Save current DESOLDER settings to EEPROM"));
                EEPROM.put(DESOLDER_EEPROM_ADDR, desolder.settings);
            }
            else if (!strcmp_P(serial_buffer, (const char *)F("read solder")))
            {
                Serial.println(F("Read from EEPROM settings for SOLDER"));
                EEPROM.get(SOLDER_EEPROM_ADDR, solder.settings);
            }
            else if (!strcmp_P(serial_buffer, (const char *)F("read desolder")))
            {
                Serial.println(F("Read from EEPROM settings for DESOLDER"));
                EEPROM.get(DESOLDER_EEPROM_ADDR, desolder.settings);
            }
            else if (!strcmp_P(serial_buffer, (const char *)F("show solder pid")))
            {
                Serial.println(F("Show internal parameters of SOLDER PID"));
                serial_print_pid(&solder);
            }
            else if (!strcmp_P(serial_buffer, (const char *)F("show desolder pid")))
            {
                Serial.println(F("Show internal parameters of DESOLDER PID"));
                serial_print_pid(&desolder);
            }
            else if (match_str(F("set solder thermo"), serial_buffer) == 17)
            {
                solder.settings.thermo_factor = atof(serial_buffer + 17);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set solder setpoint"), serial_buffer) == 19)
            {
                solder.set_point = atof(serial_buffer + 19);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set solder holdpoint"), serial_buffer) == 20)
            {
                solder.settings.hold_point = atof(serial_buffer + 20);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set solder max"), serial_buffer) == 14)
            {
                solder.settings.max = atof(serial_buffer + 14);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set solder min"), serial_buffer) == 14)
            {
                solder.settings.min = atof(serial_buffer + 14);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set solder Kp"), serial_buffer) == 13)
            {
                solder.settings.Kp = atof(serial_buffer + 13);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set solder Kd"), serial_buffer) == 13)
            {
                solder.settings.Kd = atof(serial_buffer + 13);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set solder Ki"), serial_buffer) == 13)
            {
                solder.settings.Ki = atof(serial_buffer + 13);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set desolder thermo"), serial_buffer) == 19)
            {
                desolder.settings.thermo_factor = atof(serial_buffer + 19);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set desolder setpoint"), serial_buffer) == 21)
            {
                desolder.set_point = atof(serial_buffer + 21);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set desolder holdpoint"), serial_buffer) == 22)
            {
                desolder.settings.hold_point = atof(serial_buffer + 22);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set desolder max"), serial_buffer) == 16)
            {
                desolder.settings.max = atof(serial_buffer + 16);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set desolder min"), serial_buffer) == 16)
            {
                desolder.settings.min = atof(serial_buffer + 16);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set desolder Kp"), serial_buffer) == 15)
            {
                desolder.settings.Kp = atof(serial_buffer + 15);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set desolder Kd"), serial_buffer) == 15)
            {
                desolder.settings.Kd = atof(serial_buffer + 15);
                Serial.println(VALUE_SET);
            }
            else if (match_str(F("set desolder Ki"), serial_buffer) == 15)
            {
                desolder.settings.Ki = atof(serial_buffer + 15);
                Serial.println(VALUE_SET);
            }
            else
                Serial.println(F("Unrecognized command"));

            serial_tmp_index = -1;
        }
        else
            serial_buffer[++serial_tmp_index] = serial_input_char;
    }

    // Update switches
    for (char i = 0; i < SW_COUNT; i++)
        SWITCH[i].update();

    // Read thermocouple sensors
    if ((millis() - last_temp_acquisition) >= MAX66575_ACQ_INTERVAL)
    {
        float tmp;
        last_temp_acquisition = millis();
        tmp = read_temperature(SOLDER_CS);
        if (!isnan(tmp))
            solder.temperature = round(tmp * solder.settings.thermo_factor);
        else
            solder.temperature = NAN;

        tmp = read_temperature(DESOLDER_CS);
        if (!isnan(tmp))
            desolder.temperature = round(tmp * desolder.settings.thermo_factor);
        else
            desolder.temperature = NAN;
    }

    // See what channel has been selected:
    prev_channel = channel;
    channel = 0;
    channel = !SWITCH[SW_UP].read();
    channel |= !SWITCH[SW_DOWN].read() << 1;
    if (prev_channel != channel)
        lcd_refresh = 1;

    switch (channel)
    {
    case CHANNEL::SOLDER:
        channel_iron(&solder);
        break;
    case CHANNEL::DESOLDER:
        channel_iron(&desolder);
        break;
    case CHANNEL::IDLE:
    default:
        channel_idle();
        break;
    }

    // Protection
    if (!solder.hold_state && channel != CHANNEL::SOLDER)
        solder.set_point = 0.0f;

    if (!desolder.hold_state && channel != CHANNEL::DESOLDER)
        desolder.set_point = 0.0f;

    // PID
    pid_compute(&solder);
    pid_compute(&desolder);
    // Write output
    analogWrite(solder.output_pin, (uint8_t)(round(solder.power)));
    analogWrite(desolder.output_pin, (uint8_t)(round(desolder.power)));
}

char lcd_center_string(char *string)
{
    if (!string)
        return 0;

    char string_length = strlen(string);

    if (!string_length || string_length > LCD_COLS)
        return 0;

    string_length = (((char)LCD_COLS - string_length) / (char)2);

    if (string_length > LCD_COLS)
        return 0;

    return string_length;
}

char lcd_center_string(const __FlashStringHelper *string)
{
    char _string[STR_BUFFER_SIZE];
    strncpy_P(_string, (const char *)string, STR_BUFFER_SIZE);
    return lcd_center_string(_string);
}

void init_spi_slave(char pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
}

float read_temperature(char cs_pin)
{
    int tmp = 0;

    digitalWrite(cs_pin, LOW);
    delay(2);
    digitalWrite(cs_pin, HIGH);
    delay(150);

    // Pull CS low start conversion
    digitalWrite(cs_pin, LOW);
    // Read MSB (bit15) and discard it
    digitalWrite(SCK_PIN, HIGH);
    delay(1);
    digitalWrite(SCK_PIN, LOW);

    // Read bits 14-0 from MAX6675
    for (char i = 14; i >= 0; i--)
    {
        digitalWrite(SCK_PIN, HIGH);
        tmp += digitalRead(SO_PIN) << i;
        digitalWrite(SCK_PIN, LOW);
    }

    digitalWrite(cs_pin, HIGH);

    // Check if thermocouple is open
    // if yes, value is incorrect
    if ((tmp & 0x04) >> 2)
        return NAN;
    // first 3 lsb are flags:
    // Open Themocouple, Device ID, Three state
    return (float)(tmp >> 3);
}

void serial_print_settings(IRON *obj)
{
    char tmp[STR_BUFFER_SIZE];
    dtostrf(obj->settings.thermo_factor, 3, 3, tmp);
    sprintf(lcd_buffer, "Thermo factor: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->temperature, 3, 3, tmp);
    sprintf(lcd_buffer, "Temperature: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.Kp, 3, 3, tmp);
    sprintf(lcd_buffer, "PID Kp: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.Ki, 3, 3, tmp);
    sprintf(lcd_buffer, "PID Ki: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.Kd, 3, 3, tmp);
    sprintf(lcd_buffer, "PID Kd: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.max, 3, 3, tmp);
    sprintf(lcd_buffer, "PID max: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.min, 3, 3, tmp);
    sprintf(lcd_buffer, "PID min: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->set_point, 3, 3, tmp);
    sprintf(lcd_buffer, "PID set point: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.hold_point, 3, 3, tmp);
    sprintf(lcd_buffer, "PID hold point: %s", tmp);
    Serial.println(lcd_buffer);
}

void serial_print_pid(IRON *obj)
{
    char tmp[12];
    dtostrf(obj->cumulative_error, 3, 3, tmp);
    sprintf(serial_buffer, "Cumulative error: %s", tmp);
    Serial.println(serial_buffer);

    dtostrf(obj->previous_error, 3, 3, tmp);
    sprintf(serial_buffer, "Previous error: %s", tmp);
    Serial.println(serial_buffer);

    dtostrf(obj->power, 3, 3, tmp);
    sprintf(serial_buffer, "Output: %s", tmp);
    Serial.println(serial_buffer);

    dtostrf(obj->settings.Kp, 3, 3, tmp);
    sprintf(serial_buffer, "PID Kp: %s", tmp);
    Serial.println(serial_buffer);

    dtostrf(obj->settings.Ki, 3, 3, tmp);
    sprintf(serial_buffer, "PID Ki: %s", tmp);
    Serial.println(serial_buffer);

    dtostrf(obj->settings.Kd, 3, 3, tmp);
    sprintf(serial_buffer, "PID Kd: %s", tmp);
    Serial.println(serial_buffer);
}

char _strlen(const char *str)
{
    if (!str)
        return -1;

    char ret = -1;
    while (str[++ret] != '\0')
        ;
    return ret;
}

char match_str(const char *str_a, const char *str_b)
{
    char len_a = _strlen(str_a), len_b = _strlen(str_b);

    if (len_a > len_b)
        return -1;

    for (char i = 0; i < len_a; i++)
    {
        if (str_a[i] != str_b[i])
            return i;
    }
    return len_a;
}

char match_str(const __FlashStringHelper *str_a, const char *str_b)
{
    char _str_a[STR_BUFFER_SIZE];
    strncpy_P(_str_a, (const char *)str_a, STR_BUFFER_SIZE);
    return match_str(_str_a, str_b);
}

void channel_idle(void)
{
    // Draw foreground
    if ((millis() - last_frame) > LCD_FRAME_TIME)
    {
        // Solder
        LCD.setCursor(3 + 9, 1);
        if (!isnan(solder.temperature))
        {
            dtostrf(solder.temperature, 3, 0, lcd_buffer);
            LCD.print(lcd_buffer);
            LCD.print((char)223);
            LCD.print("C");
            LCD.setCursor(19, 1);
            if (solder.hold_state)
                LCD.print('H');
        }
        else
            LCD.print(F("UNPLG"));

        // Desolder
        LCD.setCursor(3 + 9, 2);
        if (!isnan(desolder.temperature))
        {
            dtostrf(desolder.temperature, 3, 0, lcd_buffer);
            LCD.print(lcd_buffer);
            LCD.print((char)223);
            LCD.print("C");
            LCD.setCursor(19, 2);
            if (desolder.hold_state)
                LCD.print('H');
        }
        else
            LCD.print(F("UNPLG"));
    }

    // Draw background
    if (lcd_refresh)
    {
        LCD.clear();
        LCD.setCursor(7, 0);
        LCD.print(F("Idle"));
        LCD.setCursor(3, 1);
        LCD.print(F("SOLDER"));
        LCD.setCursor(3, 2);
        LCD.print(F("DESOLDER"));
        lcd_refresh = 0;
    }
}

void channel_iron(IRON *obj)
{
    char tmp[12];
    // Buttons Addition
    if (!SWITCH[SW_R].read() && (millis() - last_button) > SW_REFRESH_INTERVAL)
    {
        if (SWITCH[SW_R].currentDuration() < FIRST_STAGE_INTERVAL)
            obj->set_point += FIRST_STAGE_DELTA;
        else if (SWITCH[SW_R].currentDuration() < SECOND_STAGE_INTERVAL)
            obj->set_point += SECOND_STAGE_DELTA;
        else
            obj->set_point += THIRD_STAGE_DETLA;

        if (obj->set_point > MAX_CELSIUS_OUTPUT)
            obj->set_point = MIN_CELSIUS_OUTPUT;

        last_button = millis();
    }
    // Subtraction
    if (!SWITCH[SW_L].read() && (millis() - last_button) > SW_REFRESH_INTERVAL)
    {
        if (SWITCH[SW_L].currentDuration() < FIRST_STAGE_INTERVAL)
            obj->set_point -= FIRST_STAGE_DELTA;
        else if (SWITCH[SW_L].currentDuration() < SECOND_STAGE_INTERVAL)
            obj->set_point -= SECOND_STAGE_DELTA;
        else
            obj->set_point -= THIRD_STAGE_DETLA;

        if (obj->set_point < MIN_CELSIUS_OUTPUT)
            obj->set_point = MIN_CELSIUS_OUTPUT;
        last_button = millis();
    }

    // Hold
    if (!SWITCH[SW_C].read() && (millis() - last_button) > SW_REFRESH_INTERVAL * 3)
    {
        if (SWITCH[SW_C].currentDuration() < FIRST_STAGE_INTERVAL)
        {
            obj->hold_state = !obj->hold_state;
            if (obj->hold_state)
            {
                obj->previous_set_point = obj->set_point;
                obj->set_point = obj->settings.hold_point;
            }
            else
                obj->set_point = obj->previous_set_point;
            lcd_refresh = 1;
        }
        else if (SWITCH[SW_C].currentDuration() < SECOND_STAGE_INTERVAL)
            ;
        else
            ;

        last_button = millis();
    }

    // Draw foreground
    if ((millis() - last_frame) > LCD_FRAME_TIME && !isnan(obj->temperature))
    {

        LCD.setCursor(12, 2);
        dtostrf(obj->set_point, 3, 0, tmp);
        LCD.print(tmp);
        LCD.print((char)223);
        LCD.print("C");

        LCD.setCursor(12, 1);
        dtostrf(obj->temperature, 3, 0, tmp);
        LCD.print(tmp);
        LCD.print((char)223);
        LCD.print("C");

        LCD.setCursor(12, 3);
        dtostrf((obj->power / obj->settings.max * 100.0f), 3, 0, lcd_buffer);
        LCD.print(lcd_buffer);
        LCD.print("%");
    }

    // Draw background
    if (lcd_refresh)
    {
        LCD.clear();
        LCD.setCursor(6, 0);
        LCD.print(obj->name);
        if (obj->hold_state)
        {
            LCD.setCursor(15, 0);
            LCD.print(F("HOLD"));
        }

        if (!isnan(obj->temperature))
        {
            LCD.setCursor(0, 1);
            LCD.print(F("Current:"));
            LCD.setCursor(0, 2);
            LCD.print(F("Set:"));
            LCD.setCursor(0, 3);
            LCD.print(F("PWR:"));
        }
        else
        {
            LCD.setCursor(6, 2);
            LCD.print(F("UNPLUGGED"));
        }

        lcd_refresh = 0;
    }
}

void pid_compute(IRON *obj)
{
    if (isnan(obj->temperature))
    {
        obj->power = 0;
        return;
    }

    unsigned long current_time = millis();
    unsigned long delta_time = current_time - obj->previous_time;

    if (delta_time >= MAX66575_ACQ_INTERVAL)
    {
        float error = (obj->set_point - obj->temperature);
        float delta_error = (error - obj->previous_error); // difference of error for derivative term

        obj->cumulative_error += error; // accumalates the error - integral term
        if (obj->cumulative_error >= obj->settings.max)
            obj->cumulative_error = obj->settings.max;
        else if (obj->cumulative_error <= obj->settings.min)
            obj->cumulative_error = obj->settings.min;

        obj->power = obj->settings.Kp * error +
                     (obj->settings.Ki * MAX66575_ACQ_INTERVAL_F) * obj->cumulative_error +
                     (obj->settings.Kd / MAX66575_ACQ_INTERVAL_F) * delta_error; // PID control compute

        if (obj->power >= obj->settings.max)
            obj->power = obj->settings.max;
        else if (obj->power <= obj->settings.min)
            obj->power = obj->settings.min;

        obj->previous_error = error;
        obj->previous_time = current_time;
    }
}
