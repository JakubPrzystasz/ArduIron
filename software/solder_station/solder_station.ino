#include "solder_station.h"

LiquidCrystal_I2C LCD(LCD_ADDR, LCD_COLS, LCD_ROWS);

IRON solder("Solder", SOLDER_CS, SOLDER_PWM_PIN, SOLDER_EEPROM_ADDR),
    desolder("Desolder", DESOLDER_CS, DESOLDER_PWM_PIN, DESOLDER_EEPROM_ADDR);
Bounce SWITCH[SW_COUNT];
TOKEN tokens[TOKEN_MAX];
char serial_buffer[STR_BUFFER_SIZE], serial_input_char, serial_tmp_index = -1;
char lcd_buffer[STR_BUFFER_SIZE], lcd_refresh = 1, channel, prev_channel;
unsigned long last_frame = 0, last_temp_acquisition = 0, last_button = 0;

void setup(void)
{
    // Initialize IO for hardware:
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

    // Initialize Serial communication:
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println(F("Initializing..."));
    Serial.println(F("To show help type: help"));

    // Initialize LCD:
    LCD.init();
    LCD.backlight();
    LCD.noCursor();
    LCD.home();
    LCD.setCursor(lcd_center_string(F("Initializing...")), 1);
    LCD.print(F("Initializing..."));

    // MAX6675 chip needs some time to stabilize:
    delay(MAX6675_INIT_DELAY);
    LCD.clear();
}

void loop(void)
{
    // Update switches
    for (char i = 0; i < SW_COUNT; i++)
        SWITCH[i].update();

    // If station is operated by switches:
    if (control == CONTROL::MANUAL)
    {
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

        if (!solder.hold_flag && channel != CHANNEL::SOLDER)
            solder.set_point = 0.0f;

        if (!desolder.hold_flag && channel != CHANNEL::DESOLDER)
            desolder.set_point = 0.0f;
    }
    else
    {
        channel_idle();
    }

    // Process serial communication commands
    if (Serial.available() && (serial_input_char = Serial.read()))
        serial_process();
}

void serial_process()
{
    if (serial_input_char != '\r')
        serial_buffer[BUFFER_INDEX(serial_tmp_index)] = serial_input_char;
    else
    {
        serial_buffer[BUFFER_INDEX(serial_tmp_index)] = 0;
        // Get tokens from input string:
        char token_count = 0, tmp = 0, token_id = 0;
        for (char i = 0; i < serial_tmp_index; i++)
        {
            if ((serial_buffer[i] >= 32 && serial_buffer[i] <= 126) && serial_buffer[i] != ' ')
            {
                if (!tmp)
                    tokens[token_id].index = i;
                tmp = 1;
            }
            else
            {
                if (tmp)
                    tokens[token_id].length = i - tokens[token_id].index, token_id++;
                tmp = 0;
            }
        }
        serial_tmp_index = 0;

        if (match_str(F("help"), tokens[0]))
        {
            Serial.println(F("Help:"));
            Serial.println(F("To show current configuration:"));
            Serial.println(F("show [solder,desolder] [settings,pid,temperature]"));
            Serial.println(F("To save/read to/from EEPROM:"));
            Serial.println(F("[save,read] [solder,desolder]"));
            Serial.println(F("To set new parameters for object:"));
            Serial.println(F("set [solder,desolder] [setpoint,holdpoint,min,max,Kp,Kd,Ki,Tau] value"));
        }
        else if (match_str(F("show"), tokens[0]))
        {
            IRON *obj = 0;
            if (match_str(F("solder"), tokens[1]))
                obj = &solder;

            if (match_str(F("desolder"), tokens[1]))
                obj = &desolder;

            if (obj)
            {
                if (match_str(F("settings"), tokens[2]))
                    serial_print_settings(obj);
                else if (match_str(F("pid"), tokens[2]))
                    serial_print_pid(obj);
                else if (match_str(F("temperature"), tokens[2]))
                    serial_print_temperature(obj);
                else
                    goto show_typo;
            }
            else
            show_typo:
                Serial.println(F("Type: show [solder,desolder] [settings,pid,temperature]"));
            return;
        }
        else if (match_str(F("save"), tokens[0]))
        {
            IRON *obj = 0;
            if (match_str(F("solder"), tokens[1]))
                obj = &solder;

            if (match_str(F("desolder"), tokens[1]))
                obj = &desolder;

            if (obj)
                obj->save_eprom();
            else
                Serial.println(F("Type: save [solder,desolder]"));
            return;
        }
        else if (match_str(F("read"), tokens[0]))
        {
            IRON *obj = 0;
            if (match_str(F("solder"), tokens[1]))
                obj = &solder;

            if (match_str(F("desolder"), tokens[1]))
                obj = &desolder;

            if (obj)
                obj->read_eeprom();
            else
                Serial.println(F("Type: read [solder,desolder]"));
            return;
        }
        else if (match_str(F("set"), tokens[0]))
        {
            IRON *obj = 0;
            if (match_str(F("solder"), tokens[1]))
                obj = &solder;

            if (match_str(F("desolder"), tokens[1]))
                obj = &desolder;

            if (obj)
            {
                if (match_str(F("setpoint"), tokens[2]) && tokens[3].length)
                {
                    obj->set_point = atof(serial_buffer + tokens[3].index);
                    Serial.print(F("New setpoint in: "));
                    Serial.println(obj->name);
                }
                else if (match_str(F("holdpoint"), tokens[2]) && tokens[3].length)
                {
                    obj->settings.hold_point = atof(serial_buffer + tokens[3].index);
                    Serial.print(F("New holdpoint in: "));
                    Serial.println(obj->name);
                }
                else if (match_str(F("max"), tokens[2]) && tokens[3].length)
                {
                    obj->settings.max = atof(serial_buffer + tokens[3].index);
                    Serial.print(F("New max in: "));
                    Serial.println(obj->name);
                }
                else if (match_str(F("min"), tokens[2]) && tokens[3].length)
                {
                    obj->settings.min = atof(serial_buffer + tokens[3].index);
                    Serial.print(F("New min in: "));
                    Serial.println(obj->name);
                }
                else if (match_str(F("Kp"), tokens[2]) && tokens[3].length)
                {
                    obj->settings.Kp = atof(serial_buffer + tokens[3].index);
                    Serial.print(F("New Kp in: "));
                    Serial.println(obj->name);
                }
                else if (match_str(F("Ki"), tokens[2]) && tokens[3].length)
                {
                    obj->settings.Ki = atof(serial_buffer + tokens[3].index);
                    Serial.print(F("New Ki in: "));
                    Serial.println(obj->name);
                }
                else if (match_str(F("Kd"), tokens[2]) && tokens[3].length)
                {
                    obj->settings.min = atof(serial_buffer + tokens[3].index);
                    Serial.print(F("New Kd in: "));
                    Serial.println(obj->name);
                }
                else if (match_str(F("raw"), tokens[2]) && tokens[3].length)
                {
                    obj->raw_output = atoi(serial_buffer + tokens[3].index);
                    Serial.print(F("New raw in: "));
                    Serial.println(obj->name);
                }
                else if (match_str(F("tau"), tokens[2]) && tokens[3].length)
                {
                    obj->settings.tau = atoi(serial_buffer + tokens[3].index);
                    Serial.print(F("New Tau in: "));
                    Serial.println(obj->name);
                }
                else
                    goto set_typo;
            }
            else
            set_typo:
                Serial.println(F("Type: set [solder,desolder] [setpoint,holdpoint,min,max,Kp,Kd,Ki,Tau] value"));
            return;
        }

        INVALID_COMMAND;
        memset(&tokens, 0, sizeof(TOKEN) * TOKEN_MAX);
    }
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

void serial_print_settings(IRON *obj)
{
    char tmp[STR_BUFFER_SIZE];
    Serial.print(F("Current settings of "));
    Serial.print(obj->name);
    Serial.print("\r\n");

    dtostrf(obj->temperature, 3, FLOAT_PRECISION, tmp);
    sprintf(lcd_buffer, "Temperature: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.Kp, 3, FLOAT_PRECISION, tmp);
    sprintf(lcd_buffer, "PID Kp: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.Ki, 3, FLOAT_PRECISION, tmp);
    sprintf(lcd_buffer, "PID Ki: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.Kd, 3, FLOAT_PRECISION, tmp);
    sprintf(lcd_buffer, "PID Kd: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.max, 3, FLOAT_PRECISION, tmp);
    sprintf(lcd_buffer, "PID max: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.min, 3, FLOAT_PRECISION, tmp);
    sprintf(lcd_buffer, "PID min: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->set_point, 3, FLOAT_PRECISION, tmp);
    sprintf(lcd_buffer, "PID set point: %s", tmp);
    Serial.println(lcd_buffer);

    dtostrf(obj->settings.hold_point, 3, FLOAT_PRECISION, tmp);
    sprintf(lcd_buffer, "PID hold point: %s", tmp);
    Serial.println(lcd_buffer);
}

void serial_print_pid(IRON *obj)
{
    char tmp[STR_BUFFER_SIZE];
    Serial.print(F("Show internal parameters of PID "));
    Serial.print(obj->name);
    Serial.print("\r\n");

    dtostrf(obj->previous_error, 3, FLOAT_PRECISION, tmp);
    sprintf(serial_buffer, "Previous error: %s", tmp);
    Serial.println(serial_buffer);

    dtostrf(obj->power, 3, FLOAT_PRECISION, tmp);
    sprintf(serial_buffer, "Output: %s", tmp);
    Serial.println(serial_buffer);

    dtostrf(obj->settings.Kp, 3, FLOAT_PRECISION, tmp);
    sprintf(serial_buffer, "PID Kp: %s", tmp);
    Serial.println(serial_buffer);

    dtostrf(obj->settings.Ki, 3, FLOAT_PRECISION, tmp);
    sprintf(serial_buffer, "PID Ki: %s", tmp);
    Serial.println(serial_buffer);

    dtostrf(obj->settings.Kd, 3, FLOAT_PRECISION, tmp);
    sprintf(serial_buffer, "PID Kd: %s", tmp);
    Serial.println(serial_buffer);
}

void serial_print_temperature(IRON *obj)
{
    char tmp[STR_BUFFER_SIZE];
    Serial.print(obj->name);
    dtostrf(obj->temperature, 3, FLOAT_PRECISION, tmp);
    sprintf(serial_buffer, " %s\r\n", tmp);
    Serial.print(serial_buffer);
}

char match_str(const __FlashStringHelper *str_a, const TOKEN token)
{
    char _str_a[STR_BUFFER_SIZE];
    strncpy_P(_str_a, (const char *)str_a, STR_BUFFER_SIZE);
    for (char i = 0; i < token.length; i++)
    {
        if (_str_a[i] != serial_buffer[i + token.index])
            return 0;
    }
    return 1;
}

void channel_idle(void)
{

    solder.read_temp();
    desolder.read_temp();
    solder.write_output();
    desolder.write_output();

    // Set to automatic or manual control
    if (!SWITCH[SW_C].read() && (millis() - last_button) > SW_REFRESH_INTERVAL)
    {
        control = (CONTROL) !(char)control;
        last_button = millis();
    }

    // Draw foreground
    if ((millis() - last_frame) > LCD_FRAME_TIME)
    {
        LCD.setCursor(14, 0);
        switch (control)
        {
        case CONTROL::AUTOMATIC:
            LCD.print("Auto");
            break;

        default:
            LCD.print("    ");
            break;
        }

        // Solder
        LCD.setCursor(3 + 9, 1);
        if (!isnan(solder.temperature))
        {
            dtostrf(solder.temperature, 3, 0, lcd_buffer);
            LCD.print(lcd_buffer);
            LCD.print((char)223);
            LCD.print("C");
            LCD.setCursor(19, 1);
            if (solder.hold_flag)
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
            if (desolder.hold_flag)
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
    obj->read_temp();
    pid_compute(obj);

    obj->write_output();

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
    if (!SWITCH[SW_C].read() && (millis() - last_button) > SW_REFRESH_INTERVAL)
    {
        if (SWITCH[SW_C].currentDuration() < FIRST_STAGE_INTERVAL)
        {
            obj->hold_flag = !obj->hold_flag;
            if (obj->hold_flag)
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
        if (obj->hold_flag)
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
    if (obj->set_point <= 35)
    {
        obj->power = 0;
        return;
    }

    if (obj->previous_read_time >= obj->previous_pid_time)
    {
        unsigned long _millis = millis();
        float error = (obj->set_point - obj->temperature);
        float delta_error = (error - obj->previous_error);
        float sample_time = (float)(_millis - obj->previous_pid_time);
        float lim_max_integ, lim_min_integ;

        obj->proportional = obj->settings.Kp * error;

        obj->integral += (0.5f * obj->settings.Kp * sample_time * (error + obj->previous_error));

        if (obj->settings.max > obj->proportional)
            lim_max_integ = obj->settings.max - obj->proportional;
        else
            lim_max_integ = 0.0f;

        if (obj->settings.min < obj->proportional)
            lim_max_integ = obj->settings.min - obj->proportional;
        else
            lim_min_integ = 0.0f;

        // Constrain Integrator
        if (obj->integral > lim_max_integ)
            obj->integral = lim_max_integ;

        else if (obj->integral < lim_min_integ)
            obj->integral = lim_min_integ;

        obj->derivative = (2.0f * obj->settings.Kd * (obj->temperature - obj->previous_temperature) + (2.0f * obj->settings.tau - sample_time) * obj->derivative) / (2.0f * obj->settings.tau + sample_time);

        obj->raw_output = constrain(obj->proportional + obj->integral + obj->derivative, obj->settings.min, obj->settings.max);

        obj->previous_error = error;
        obj->previous_temperature = obj->temperature;
        obj->previous_pid_time = _millis;
    }
}
