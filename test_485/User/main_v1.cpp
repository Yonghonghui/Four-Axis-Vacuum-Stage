#include "./c_bindings.h"


class Motor {
private:
    uint8_t addr;             // Motor address
    uint8_t set_dir;          // Set forward direction (0 or 1)
    uint8_t dir;              // Current direction bit sent to the motor
    uint8_t redu_ratio;       // Reduction ratio of the motor gearbox
    uint8_t acc;              // Acceleration parameter for the motor
    uint16_t vel;             // Target velocity (raw value to be sent to motor)

public:
    int velocity;             // User-defined speed (RPM)
    double tgt_degree;        // Target angle (degrees)
    int32_t read_velocity;    // Real-time read velocity (RPM)
    int32_t read_position_raw;// Real-time read raw position count (signed)
    double read_degree;       // Real-time read angle (degrees)
    bool reach_pos;

    // Initializes the motor address, direction, reduction ratio, and acceleration
    void init(uint8_t address, uint8_t direction = 0, uint8_t reduction_ratio = 1, uint8_t acc_val = 10) {
        addr = address;
        set_dir = direction;
        redu_ratio = reduction_ratio;
        acc = acc_val;
        velocity = 0;
        tgt_degree = 0;
        read_velocity = 0;
        read_position_raw = 0;
        read_degree = 0;
    }

    // Calculates the absolute position count value after setting the target angle
    uint32_t read_tgt_pos() const {
        return static_cast<uint32_t>(tgt_degree * (3200.0 * redu_ratio) / 360.0);
    }

    // Sets the target position (in degrees) and optionally the velocity, then sends the position control command
    uint32_t tgt_position(double degree, int velocity_val = 1) { // [/3200 = round] - Comment likely referring to a rounding aspect in the underlying implementation
        tgt_degree = degree;
        if (degree < 0) {
            dir = set_dir ? 0 : 1; // Reverse direction if the degree is negative
            degree = -degree; // Store the absolute value of the degree
        } else {
            dir = set_dir;
        }
        uint32_t position = degree * (3200 * redu_ratio) / 360; // [/3200 = round] - Comment likely referring to a rounding aspect in the underlying implementation
        if (velocity_val!=0) {
            vel = static_cast<uint16_t>(velocity_val); // [RPM] - Set velocity if a velocity value is provided
        }
        Emm_V5_Pos_Control(addr, dir, vel, acc, position, 0, 0); // [degree] - Send position control command to the motor
        return position;
    }

    // Sets the velocity and direction based on the input velocity value, without sending a command
    void set_velocity(int velocity_val) {
        velocity = velocity_val;
        if (velocity < 0) {
            vel = static_cast<uint16_t>(-velocity); // Store the absolute value of the negative velocity
            dir = set_dir ? 0 : 1;                // Reverse direction if the velocity is negative
        } else {
            vel = static_cast<uint16_t>(velocity);  // Store the positive velocity
            dir = set_dir;                         // Maintain the set forward direction
        }
    }

    // Sends the constant speed control command to the motor
    void constant_rorate() {
        Emm_V5_Vel_Control(addr, dir, vel, acc, 0);
    }

    void constant_rorate(int velocity_val) {
        set_velocity(velocity_val);
        Emm_V5_Vel_Control(addr, dir, vel, acc, 0);
    }

    // Returns the reduction ratio of the motor
    uint8_t get_reduction_ratio() const { return redu_ratio; }

    // Displays the current status (speed, target angle, working status) on the TFT screen
    void displayStatus(int baseX, int baseY) const {
        char buf[32];
        const int offsetX = 10;    // Horizontal spacing
        int currentY = baseY;
        int currentX = baseX;

        // --- First line: Motor label and status (STOP/RUN) ---
        // Output status with a fixed width of 4 characters
        snprintf(buf, sizeof(buf), "M%02X:", addr);
        lcd_show_string(currentX, currentY, 60, 16, 16, buf, BLUE);
        currentX += 60 + offsetX * 2;

        const char* status = (velocity != 0) ? "RUN"  : "STOP";
        uint16_t  color  = (velocity != 0) ? GREEN : RED;
        // Fixed width of 4 characters, padded with spaces on the right
        snprintf(buf, sizeof(buf), "%-4s", status);
        lcd_show_string(currentX, currentY, 60, 16, 16, buf, color);

        // // --- Second line: SPEED and DIRECTION or STOP "0" ---
        // currentY += 20;
        // currentX = baseX;
        // lcd_show_string(currentX, currentY, 60, 16, 16, "SPEED:", BLUE);
        // currentX += 60 + offsetX;

        // if (velocity > 0) {
        //     // Positive rotation: Direction string fixed width 5 characters, right-padded + number fixed width 2 digits
        //     const char* dirStr = (set_dir == 0) ? "CLW"  : "CCLW"; // Clockwise or Counter-Clockwise
        //     // "CLW  " occupies 5 columns; the following number occupies 2 columns
        //     snprintf(buf, sizeof(buf), "%-5s%2d", dirStr, velocity);
        //     lcd_show_string(currentX, currentY, 100, 16, 16, buf, BLUE);
        // } else if (velocity < 0) {
        //     // Reverse rotation
        //     const char* dirStr = (set_dir == 0) ? "CCLW" : "CLW";
        //     snprintf(buf, sizeof(buf), "%-5s%2d", dirStr, -velocity);
        //     lcd_show_string(currentX, currentY, 100, 16, 16, buf, BLUE);
        // } else {
        //     // STOP: Directly output a fixed-width string, overwriting the direction + old number
        //     // Assuming a total width of 7 characters here: 6 spaces + "0"
        //     lcd_show_string(currentX, currentY, 100, 16, 16, "       0", BLUE);
        // }
        // --- Second line: SPEED and DIRECTION or STOP "0" ---
        currentY += 20;
        currentX = baseX;
        lcd_show_string(currentX, currentY, 60, 16, 16, "SPEED:", BLUE);
        currentX += 60 + offsetX;

        if (velocity > 0) {
            const char* dirStr = (set_dir == 0) ? "CLW"  : "CCLW";
            char velBuf[3]; // 假设速度不会超过两位数 (例如 99 RPM)
            snprintf(velBuf, sizeof(velBuf), "%d", velocity);
            char displayBuf[10];
            snprintf(displayBuf, sizeof(displayBuf), "%-5s%s", dirStr, velBuf);
            lcd_show_string(currentX, currentY, 70, 16, 16, displayBuf, BLUE);
        } else if (velocity < 0) {
            const char* dirStr = (set_dir == 0) ? "CCLW" : "CLW";
            char velBuf[3]; // 假设速度绝对值不会超过两位数
            snprintf(velBuf, sizeof(velBuf), "%d", -velocity);
            char displayBuf[10];
            snprintf(displayBuf, sizeof(displayBuf), "%-5s%s", dirStr, velBuf);
            lcd_show_string(currentX, currentY, 70, 16, 16, displayBuf, BLUE);
        } else {
            lcd_show_string(currentX, currentY, 70, 16, 16, "       0", BLUE);
        }

        // --- Third line: TARGET DEGREE ---
        currentY += 20;
        currentX = baseX;
        lcd_show_string(currentX, currentY, 100, 16, 16, "TARGET:", BLUE);
        currentX += 100 + offsetX;
        // Target angle fixed width 5 digits (including sign)
        int tgt_int = static_cast<int>(tgt_degree);
        snprintf(buf, sizeof(buf), "%5d", tgt_int);
        lcd_show_string(currentX, currentY, 80, 16, 16, buf, BLUE);
    }
};

// Global definitions for four motor instances
Motor motor1, motor2, motor3, motor4;

// Parses the received RS485 data, updates the corresponding motor, and refreshes the display
void Translate_received_data(uint8_t* rs485buf) {
    uint8_t len;
    rs485_receive_data(rs485buf, &len);
    if (len == 0) return;
    if (len > 8) len = 8;

    uint8_t motor_addr    = rs485buf[0];
    uint8_t function_code = rs485buf[1];
    Motor* pm;
    switch (motor_addr) {
        case 1: pm = &motor1; break;
        case 2: pm = &motor2; break;
        case 3: pm = &motor3; break;
        case 4: pm = &motor4; break;
        default: return;
    }

    switch (function_code) {
        case 0x35: // Velocity feedback
            if (len >= 6) {
                uint8_t sign = rs485buf[2];
                uint16_t speed_raw = (rs485buf[3] << 8) | rs485buf[4];
                pm->read_velocity = (sign == 0x01) ? -static_cast<int32_t>(speed_raw) : speed_raw;
            }
            break;
        case 0x36: // Position feedback
            if (len >= 8) {
                uint8_t sign = rs485buf[2];
                uint32_t pos = (rs485buf[3] << 24) | (rs485buf[4] << 16) |
                               (rs485buf[5] << 8)  | rs485buf[6];
                pm->read_position_raw = (sign == 0x01) ? -static_cast<int32_t>(pos) : pos;
                uint8_t rr = pm->get_reduction_ratio();
                pm->read_degree = pm->read_position_raw * 360.0 / (3200.0 * rr);
            }
            break;
        case 0x3A: // Status flag
            if (len >= 4) {
                uint8_t status = rs485buf[2];
                pm->reach_pos = (status & 0x02) ? true : false; // Check if the target position is reached
            }
            break;
        default:
            break;
    }

    // The four motors are arranged vertically, each refreshed individually
    int baseX = 30;
    for (uint8_t i = 0; i < 4; ++i) {
        int y = 210 + i * 100;
        switch (i + 1) {
            case 1: motor1.displayStatus(baseX, y); break;
            case 2: motor2.displayStatus(baseX, y); break;
            case 3: motor3.displayStatus(baseX, y); break;
            case 4: motor4.displayStatus(baseX, y); break;
        }
    }
}

void move_set_1() {
    motor1.tgt_position(15, 15); // [degree]
    delay_ms(10);
    motor2.tgt_position(30, 10); // [mm]
    delay_ms(10);
    motor3.tgt_position(40, 5); // [degree]
    delay_ms(10);
    motor4.constant_rorate(10);
    delay_ms(10);
    
    for(int i = 0; i < 30; i++) {
        HAL_Delay(1000);
    }

    motor4.constant_rorate(100); // [RPM]
    delay_ms(10);

    motor1.tgt_position(5, 3);
    delay_ms(10);
    motor2.tgt_position(10, 2);
    delay_ms(10);
    motor3.tgt_position(-20, 5);
    delay_ms(10);

    for(int i = 0; i < 60; i++) {
        HAL_Delay(1000);
    }

	motor4.constant_rorate(10); // [RPM]
    delay_ms(10);
	//back to home
    motor1.tgt_position(-20, 30);
    delay_ms(10);
    motor2.tgt_position(-40, 20);
    delay_ms(10);
    motor3.tgt_position(-20, 10);
    delay_ms(10);
	motor4.constant_rorate(0); // [RPM]
    delay_ms(10);


    // for(int i = 0; i < 30; i++) {
    //     HAL_Delay(1000);
    // }
    // Emm_V5_Stop_Now(1, 0);
    // delay_ms(10);
    // Emm_V5_Stop_Now(2, 0);
    // delay_ms(10);
    // Emm_V5_Stop_Now(3, 0);
    // delay_ms(10);
    // Emm_V5_Stop_Now(4, 0);
    // delay_ms(10);
}

void move_set_2() {
    motor1.tgt_position(-20,10);
    delay_ms(10);
    motor2.tgt_position(-40,10);
    delay_ms(10);
    motor3.tgt_position(-10, 10);
    delay_ms(10);
}

int main(void) {
    uint8_t key, t = 0, cnt = 0;
    uint8_t rs485buf[8];

    HAL_Init();
    sys_stm32_clock_init(336, 8, 2, 7);
    delay_init(168);
    usart_init(115200);
    usmart_dev.init(84);
    led_init();
    lcd_init();
    key_init();
    rs485_init(115200);

    lcd_show_string(30, 50, 200, 16, 16, "STM32", RED);
    lcd_show_string(30, 70, 200, 16, 16, "RS485 TEST", RED);

    // Motor initialization
    //(addr, dir, redu_ratio, acc)
    motor1.init(1, 1, 100, 128); //(1,1,100,128)
    motor2.init(2, 1, 30, 128);
    motor3.init(3, 0, 10, 64);
    motor4.init(4, 0, 1, 1);

    // motor1.init(1, 0, 1, 128);
    // motor2.init(2, 0, 1, 128);
    // motor3.init(3, 0, 1, 64);
    // motor4.init(4, 0, 1, 16);


    while (1) {
        key = key_scan(0);

        if(key == KEY0_PRES) {
            lcd_show_string(30, 150, 500, 100, 16, "KEY0 Pressed", BLUE);
            move_set_1();
            delay_ms(10);
        }

        if(key == KEY1_PRES) {
            lcd_show_string(30, 150, 500, 100, 16, "KEY1 Pressed", BLUE);
            move_set_2();
            delay_ms(10);
        }


        if(key == KEY2_PRES) {
            lcd_show_string(30, 150, 500, 100, 16, "stop", BLUE);
            Emm_V5_Stop_Now(1, 0);
            delay_ms(10);
            Emm_V5_Stop_Now(2, 0);
            delay_ms(10);
            Emm_V5_Stop_Now(3, 0);
            delay_ms(10);
            Emm_V5_Stop_Now(4, 0);
            delay_ms(10);
        }

        // if (key == KEY0_PRES) {
        //     lcd_show_string(30, 150, 500, 100, 16, "Sequence 1 ", BLUE);
        //     // motor1.velocity += 1;
        //     // motor1.set_velocity(motor1.velocity); // [RPM]
        //     // motor1.constant_rorate();
        //     // delay_ms(10);
        //     motor2.velocity += 1;
        //     motor2.set_velocity(motor2.velocity); // [RPM]
        //     motor2.constant_rorate();
        //     delay_ms(10);
        //     // motor3.velocity += 1;
        //     // motor3.set_velocity(motor3.velocity); // [RPM]
        //     // motor3.constant_rorate();
        //     // delay_ms(10);
        //     // motor4.velocity += 1;
        //     // motor4.set_velocity(motor4.velocity); // [RPM]
        //     // motor4.constant_rorate();

        // }

        // if (key == KEY1_PRES) {
        //     lcd_show_string(30, 150, 500, 100, 16, "Sequence 2 ", BLUE);
        //     // motor1.velocity -= 1;
        //     // motor1.set_velocity(motor1.velocity); // [RPM]
        //     // motor1.constant_rorate();
        //     // delay_ms(10);
        //     motor2.velocity -= 1;
        //     motor2.set_velocity(motor2.velocity); // [RPM]
        //     motor2.constant_rorate();
        //     delay_ms(10);
        //     // motor3.velocity -= 1;
        //     // motor3.set_velocity(motor3.velocity); // [RPM]
        //     // motor3.constant_rorate();
        //     // delay_ms(10);
        //     // motor4.velocity -= 1;
        //     // motor4.set_velocity(motor4.velocity); // [RPM]
        //     // motor4.constant_rorate();
        // }
        // if (key == KEY2_PRES) {
        //     // When KEY2 is pressed, make motor1/2/3 move to the specified positions with speed parameters
        //     // motor1 → 180° @ 30 RPM; motor2 → 60° @ -15 RPM; motor3 → 180° @ 20 RPM
        //     lcd_show_string(30, 150, 500, 100, 16, "Sequence 3 ", BLUE);
        //     // motor1.set_target_position(180.0, 10);
        //     // delay_ms(10);
        //     // motor2.set_target_position(90.0, 15);
        //     // delay_ms(10);
        //     // motor3.set_target_position(180.0, 20);
        //     // To remove the delay for simultaneous start, delete the intermediate delay_ms(10);
        //     motor1.set_velocity(0);
        //     motor1.constant_rorate();
        //     // delay_ms(10);
        //     // motor2.set_velocity(0);
        //     // motor2.constant_rorate();
        //     // delay_ms(10);
        //     // motor3.set_velocity(0);
        //     // motor3.constant_rorate();
        //     // delay_ms(10);
        //     // motor4.set_velocity(0);
        //     // motor4.constant_rorate();
        // }


        delay_ms(10);

        if (++t >= 20) {
            t = 0;
            LED0_TOGGLE();
            cnt++;
            lcd_show_xnum(78, 130, cnt, 3, 16, 0x80, BLUE);
            for (uint8_t addr = 1; addr <= 4; ++addr) {
                Emm_V5_Read_Sys_Params(addr, S_VEL);  // Read velocity feedback
                Emm_V5_Read_Sys_Params(addr, S_CPOS); // Read current position feedback
                Emm_V5_Read_Sys_Params(addr, S_FLAG);
            }
        }

        Translate_received_data(rs485buf);
        delay_ms(10);
    }
    return 0;
}