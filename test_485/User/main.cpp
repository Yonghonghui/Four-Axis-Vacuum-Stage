#include "./c_bindings.h"
#include <math.h>   // or <cmath>

// --- State Machine Definition ---
enum MoveState1 {
    MS1_IDLE = 0,
    MS1_STEP1,   // Initial move
    MS1_WAIT1,   // Wait for 30s
    MS1_STEP2,   // Accelerate
    MS1_WAIT2,   // Wait for 60s
    MS1_STEP3,   // Decelerate and return to initial
    MS1_WAIT3,   // Wait for the motors to stop
    MS1_DONE     // Done, can be reset
};

enum MoveState2 {
    MS2_IDLE = 0,
    MS2_STEP1,   // Initial move
    MS2_WAIT1,   // Wait for 30s
    MS2_STEP2,   // Accelerate
    MS2_WAIT2,   // Wait for 60s
    MS2_STEP3,   // Decelerate and return to initial
    MS2_WAIT3,   // Wait for the motors to stop
    MS2_DONE     // Done, can be reset
};

static MoveState1 move1_state = MS1_IDLE;
static MoveState2 move2_state = MS2_IDLE;
static uint32_t move1_lastTick = 0;

const char* MoveState1Names[] = {
    "IDLE",
    "STEP1",
    "WAIT1",
    "STEP2",
    "WAIT2",
    "STEP3",
    "WAIT3",
    "DONE"
};

// Define the message display area size
#define MSG_X      30
#define MSG_Y      150
#define MSG_W      500
#define MSG_H      100
// Define a unified output function that prints with fixed width and pads spaces automatically
void displayMessage(const char* msg) {
    // Assume the message area occupies a total of 20 characters in width
    char buf[32];
    snprintf(buf, sizeof(buf), "%-20s", msg);  
    // "%-20s": left-aligned, total of 20 characters, padded with spaces

    lcd_show_string(MSG_X, MSG_Y, MSG_W, MSG_H, 16, buf, BLUE);
}


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
    int duration; // Duration for the motor to reach the target position
    char* status; // Status string for the motor

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
        status = "STOP";
    
    }

    // // Calculates the absolute position count value after setting the target angle
    // uint32_t read_tgt_pos() const {
    //     return static_cast<uint32_t>(tgt_degree * (3200.0 * redu_ratio) / 360.0);
    // }

    // Sets the target position (in degrees) and optionally the velocity, then sends the position control command
    uint32_t tgt_position(double degree, uint16_t velocity_val = 1) { // [/3200 = round] - Comment likely referring to a rounding aspect in the underlying implementation
        tgt_degree = degree;
        read_degree += degree; // Update the real-time read angle
        if ((velocity_val == 1 && vel == 0) || (velocity_val != 1)) {
            vel = velocity_val; // [RPM] - Set velocity if a velocity value is provided
        }
        if (degree < 0) {
            dir = set_dir ? 0 : 1; // Reverse direction if the degree is negative
            degree = -degree; // Store the absolute value of the degree
            velocity = -vel;
        } else {
            dir = set_dir;
            velocity = vel;
        }
        uint32_t position = degree * 3200 * redu_ratio /360; // [/3200 = round] - Comment likely referring to a rounding aspect in the underlying implementation
        duration = degree * redu_ratio *60000 /360 / vel +1000; //[ms] // Calculate the duration to reach the target position

        Emm_V5_Pos_Control(addr, dir, vel, acc, position, 0, 0); // [degree] - Send position control command to the motor
        return position;
    }

    // Sets the velocity and direction based on the input velocity value, without sending a command
    // Sets the velocity and direction based on the input velocity value, without sending a command
    void set_velocity(int velocity_val) {
        velocity = velocity_val;
        if (velocity < 0) {
            vel = static_cast<uint16_t>(-velocity); // Store the absolute value of the negative velocity
            dir = set_dir ? 0 : 1;                  // Reverse direction if the velocity is negative
        } else {
            vel = static_cast<uint16_t>(velocity);  // Store the positive velocity
            dir = set_dir;                          // Maintain the set forward direction
        }
    }

    // Sends the constant speed control command to the motor
    void constant_rorate() {
        Emm_V5_Vel_Control(addr, dir, vel, acc, 0);
    }

    void constant_rorate(int velocity_val) {
        if(velocity_val == 0) {
            status = "STOP";
        }
        set_velocity(velocity_val);
        Emm_V5_Vel_Control(addr, dir, vel, acc, 0);
    }

    // Returns the reduction ratio of the motor
    // Returns the reduction ratio of the motor
    uint8_t get_reduction_ratio() const { return redu_ratio; }

    // Displays the current status (speed, target angle, working state) on the TFT screen
    void displayStatus(int baseX, int baseY) const {
        char buf[32];
        const int offsetX = 10;    // Horizontal spacing
        int currentY = baseY;
        int currentX = baseX;

        // --- First line: Motor label and status (STOP/RUN) ---
        // Output the label and state with fixed width
        snprintf(buf, sizeof(buf), "M%02X:", addr);
        lcd_show_string(currentX, currentY, 60, 16, 16, buf, BLUE);
        currentX += 60 + offsetX * 2;

        // const char* status = (velocity != 0) ? "RUN"  : "STOP";
        // uint16_t  color  = (velocity != 0) ? GREEN : RED;
        // // Fixed width of 4 characters, padded with spaces on the right
        // snprintf(buf, sizeof(buf), "%-4s", status);
        // lcd_show_string(currentX, currentY, 60, 16, 16, buf, color);

        //status
        uint16_t color = (status[0] == 'R') ? GREEN : RED;
        snprintf(buf, sizeof(buf), "%-4s", status);
        lcd_show_string(currentX, currentY, 60, 16, 16, buf, color);

        // --- Second line (continued): SPEED and DIRECTION or STOP "0" ---
        currentY += 20;
        currentX = baseX;
        lcd_show_string(currentX, currentY, 60, 16, 16, "SPEED:", BLUE);
        currentX += 60 + offsetX;

        char displayBuf[10];
        if (velocity > 0) {
            const char* dirStr = (set_dir == 0) ? "CLW"  : "CCLW";
            // Total fixed length of 8 characters: 4 for direction + 3 for speed + 1 for trailing space
            snprintf(displayBuf, sizeof(displayBuf), "%-4s%3d ", dirStr, velocity);
        } else if (velocity < 0) {
            const char* dirStr = (set_dir == 0) ? "CCLW" : "CLW";
            snprintf(displayBuf, sizeof(displayBuf), "%-4s%3d ", dirStr, -velocity);
        } else {
            // Speed is 0: fixed 8 characters, with number right aligned
            snprintf(displayBuf, sizeof(displayBuf), "    0   ");
        }
        // Directly write on screen; trailing spaces overwrite old characters
        lcd_show_string(currentX, currentY, 70, 16, 16, displayBuf, BLUE);

        // --- Third line: TARGET DEGREE ---
        currentY += 20;
        currentX = baseX;
        lcd_show_string(currentX, currentY, 100, 16, 16, "TARGET:", BLUE);
        currentX += 100 + offsetX;
        // Target angle printed with fixed width of 5 digits (including sign)
        int tgt_int = static_cast<int>(tgt_degree);
        snprintf(buf, sizeof(buf), "%5d", tgt_int);
        lcd_show_string(currentX, currentY, 80, 16, 16, buf, BLUE);
    }
};

// Global definitions for four motor instances
Motor motor[5];

// Track the status for four motors
static uint32_t motorStartTick[4]   = {0, 0, 0, 0};  // Each motor's movement start tick (HAL_GetTick())
static bool     motorMoving[4]      = {false, false, false, false}; // Indicates if the motor is in motion
int wait_time[5] = {0,0,0,0,0}; //[s] // Wait time for each motor

// void pollMotorStops() {
//     uint32_t now = HAL_GetTick();
//     for (int i = 0; i < 4; i++) {
//         if (motorMoving[i] && now - motorStartTick[i] >= motorDuration[i]) {
//             // Stop the corresponding motor
//             motor[i].constant_rorate(0);
//             motorMoving[i] = false;    // Clear the movement flag
//         }
//     }
// }

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
        case 1: pm = &motor[0]; break;
        case 2: pm = &motor[1]; break;
        case 3: pm = &motor[2]; break;
        case 4: pm = &motor[3]; break;
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
    for (uint8_t i = 0; i < 4; i++) {
        int y = 210 + (i + 1) * 100;
        motor[i].displayStatus(baseX, y);
    }
}

// --- Non-blocking progression function ---
void process_move_set_1(void) {
    uint32_t now = HAL_GetTick();
    switch (move1_state) {
        case MS1_IDLE:
            // Idle - waiting for trigger
            break;

        case MS1_STEP1:
            // Step 1: All four motors act simultaneously
            motor[0].tgt_position(18, 40); //[degree]
            motorStartTick[0] = HAL_GetTick();
            motorMoving[0]    = true;
            delay_ms(10);
            motor[1].tgt_position(30, 30); //[mm]
            motorStartTick[1] = HAL_GetTick();
            motorMoving[1]    = true;
            delay_ms(10);
            motor[2].tgt_position(5, 10); //[degree]
            motorStartTick[2] = HAL_GetTick();
            motorMoving[2]    = true;
            delay_ms(10);
            motor[3].constant_rorate(3);
            delay_ms(10);
            for(int i = 0; i < 4; i++) {
                motor[i].status = "RUN";
            }

            for(int i = 0; i < 3; i++) 
                if(motor[i].duration > wait_time[0]*1000) {
                    wait_time[0] = motor[i].duration/1000;
                }

            move1_lastTick = now;
            move1_state = MS1_WAIT1;
            break;

        case MS1_WAIT1:
            // Wait for 30 seconds (30000 ms)
            for(int i = 0; i < 3; i++) {
                if (now - motorStartTick[i] >= motor[i].duration) {
                    motor[i].status = "STOP";
                    motor[i].duration = 0;
                }    
            }
            if (now - move1_lastTick >= wait_time[0]*1000) {
                move1_state = MS1_STEP2;
            }
            break;

        case MS1_STEP2:
            // Step 2: Raise the platform
            motor[0].tgt_position(-5, 5);
            motorStartTick[0] = HAL_GetTick();
            motorMoving[0]    = true;
            delay_ms(10);
            motor[1].tgt_position(-20, 2);
            motorStartTick[1] = HAL_GetTick();
            motorMoving[1]    = true;
            delay_ms(10);
            motor[2].tgt_position(30, 5);
            motorStartTick[2] = HAL_GetTick();
            motorMoving[2]    = true;
            delay_ms(10);
            for(int i = 0; i < 4; i++) {
                motor[i].status = "RUN";
            }
            for(int i = 0; i < 3; i++) 
                if(motor[i].duration > wait_time[1]*1000) {
                    wait_time[1] = motor[i].duration/1000;
                }

            move1_lastTick = now;
            move1_state = MS1_WAIT2;
            break;

        case MS1_WAIT2:
            // Wait another 60 seconds (60000 ms)
            for(int i = 0; i < 3; i++) {
                if (now - motorStartTick[i] >= motor[i].duration) {
                    motor[i].status = "STOP";
                    motor[i].duration = 0;
                }    
            }
            if (now - move1_lastTick >= wait_time[1]*1000) {
                move1_state = MS1_STEP3;
            }

            break;

        case MS1_STEP3:
            // Step 3: Return to home
            motor[0].tgt_position(-13, 40);
            motorStartTick[0] = HAL_GetTick();
            motorMoving[0]    = true;
            delay_ms(10);
            motor[1].tgt_position(-10, 8);
            motorStartTick[1] = HAL_GetTick();
            motorMoving[1]    = true;
            delay_ms(10);
            motor[2].tgt_position(-35, 3);
            motorStartTick[2] = HAL_GetTick();
            motorMoving[2]    = true;
            delay_ms(10);
            motor[3].constant_rorate(0);
            delay_ms(10);
				    for(int i = 0; i < 3; i++) {
                motor[i].status = "RUN";
            }
            
            move1_lastTick = now;
            wait_time[2] = 0; // Reset wait time for the next step
            for(int i = 0; i < 3; i++) 
                if(motor[i].duration > wait_time[2]*1000) {
                    wait_time[2] = motor[i].duration/1000;
                }
            wait_time[2] += 5; // Add a 5-second buffer to the wait time
            motor[3].status = "STOP";
            move1_state = MS1_WAIT3;
            break;

        case MS1_WAIT3:
            // Wait for the motors to stop
            for(int i = 0; i < 3; i++) {
                if (now - motorStartTick[i] >= motor[i].duration) {
                    motor[i].status = "STOP";
                    motor[i].duration = 0;
                }    
            }
            if (now - move1_lastTick >= wait_time[2]*1000) {
                move1_state = MS1_DONE;
            }
            break;

        case MS1_DONE:
            // After completion, either automatically return to IDLE or remain DONE until Key0 re-triggers
            for(int i = 0; i < 4; i++) {
                motor[i].status = "STOP";
            }
            move1_state = MS1_IDLE;
            break;
    }
}

void process_move_set_2(void) {
    uint32_t now = HAL_GetTick();
    switch (move2_state) {
        case MS2_IDLE:
            // Idle - waiting for trigger
            break;

        case MS2_STEP1:
            // Step 1: All four motors act simultaneously
            motor[0].tgt_position(18, 50); //[degree]
            motorStartTick[0] = HAL_GetTick();
            motorMoving[0]    = true;
            delay_ms(10);
            motor[1].tgt_position(30, 40); //[mm]
            motorStartTick[1] = HAL_GetTick();
            motorMoving[1]    = true;
            delay_ms(10);
            motor[2].tgt_position(5, 15); //[degree]
            motorStartTick[2] = HAL_GetTick();
            motorMoving[2]    = true;
            delay_ms(10);
            motor[3].constant_rorate(50);
            delay_ms(10);
            for(int i = 0; i < 4; i++) {
                motor[i].status = "RUN";
            }

            for(int i = 0; i < 3; i++) 
                if(motor[i].duration > wait_time[0]*1000) {
                    wait_time[0] = motor[i].duration/1000;
                }

            move1_lastTick = now;
            move2_state = MS2_WAIT1;
            break;

        case MS2_WAIT1:
            // Wait for 30 seconds (30000 ms)
            for(int i = 0; i < 3; i++) {
                if (now - motorStartTick[i] >= motor[i].duration) {
                    motor[i].status = "STOP";
                    motor[i].duration = 0;
                }    
            }
            if (now - move1_lastTick >= wait_time[0]*1000) {
                move2_state = MS2_STEP2;
            }
            break;

        case MS2_STEP2:
            // Step 2: Raise the platform
            motor[0].tgt_position(-5, 10);
            motorStartTick[0] = HAL_GetTick();
            motorMoving[0]    = true;
            delay_ms(10);
            motor[1].tgt_position(-20, 5);
            motorStartTick[1] = HAL_GetTick();
            motorMoving[1]    = true;
            delay_ms(10);
            motor[2].tgt_position(50, 5);
            motorStartTick[2] = HAL_GetTick();
            motorMoving[2]    = true;
            delay_ms(10);
            for(int i = 0; i < 4; i++) {
                motor[i].status = "RUN";
            }
            for(int i = 0; i < 3; i++) 
                if(motor[i].duration > wait_time[1]*1000) {
                    wait_time[1] = motor[i].duration/1000;
                }

            move1_lastTick = now;
            move2_state = MS2_WAIT2;
            break;

        case MS2_WAIT2:
            // Wait another 60 seconds (60000 ms)
            for(int i = 0; i < 3; i++) {
                if (now - motorStartTick[i] >= motor[i].duration) {
                    motor[i].status = "STOP";
                    motor[i].duration = 0;
                }    
            }
            if (now - move1_lastTick >= wait_time[1]*1000) {
                move2_state = MS2_STEP3;
            }

            break;

        case MS2_STEP3:
            // Step 3: Return to home
            motor[0].tgt_position(-13, 50);
            motorStartTick[0] = HAL_GetTick();
            motorMoving[0]    = true;
            delay_ms(10);
            motor[1].tgt_position(-10, 15);
            motorStartTick[1] = HAL_GetTick();
            motorMoving[1]    = true;
            delay_ms(10);
            motor[2].tgt_position(-55, 15);
            motorStartTick[2] = HAL_GetTick();
            motorMoving[2]    = true;
            delay_ms(10);
            motor[3].constant_rorate(0);
            delay_ms(10);
			for(int i = 0; i < 3; i++) {
                motor[i].status = "RUN";
            }
            
            move1_lastTick = now;
            wait_time[2] = 0; // Reset wait time for the next step
            for(int i = 0; i < 3; i++) 
                if(motor[i].duration > wait_time[2]*1000) {
                    wait_time[2] = motor[i].duration/1000;
                }
            wait_time[2] += 5; // Add a 5-second buffer to the wait time
            motor[3].status = "STOP";
            move2_state = MS2_WAIT3;
            break;

        case MS2_WAIT3:
            // Wait for the motors to stop
            for(int i = 0; i < 3; i++) {
                if (now - motorStartTick[i] >= motor[i].duration) {
                    motor[i].status = "STOP";
                    motor[i].duration = 0;
                }    
            }
            if (now - move1_lastTick >= wait_time[2]*1000) {
                move2_state = MS2_DONE;
            }
            break;

        case MS2_DONE:
            // After completion, either automatically return to IDLE or remain DONE until Key0 re-triggers
            for(int i = 0; i < 4; i++) {
                motor[i].status = "STOP";
            }
            move2_state = MS2_IDLE;
            break;
    }
}

void manual_set_1() {
    motor[0].tgt_position(0, 30);
    delay_ms(10);
    motor[1].tgt_position(3, 10);
    delay_ms(10);
    motor[2].tgt_position(-1, 20);
    delay_ms(10);
    // motor[3].tgt_position(20,10);
}

int main(void) {
    uint8_t key, t = 0, cnt = 0;
    uint8_t rs485buf[8];
    // char stateBuf[16];      // Declare buffer here

    HAL_Init();
    sys_stm32_clock_init(336, 8, 2, 7);
    delay_init(168);
    usart_init(115200);
    led_init();
    lcd_init();
    key_init();
    rs485_init(115200);

    // x, y, w, h, font_size
    lcd_show_string(
        30, 50, 200, 16, 16, 
        "Senior Design:", RED
    );
    lcd_show_string(
        30, 70, 320, 16, 16, 
        "Four-Axis Vacuum Stage", RED
    );
    lcd_show_string(
        30, 90, 320, 16, 16, 
        "for Advanced Nano-Manufacturing", RED
    );

    // Motor initialization
    // addr, dir, redu_ratio, acc
    motor[0].init(1, 1, 100, 128);
    motor[1].init(2, 1, 30, 128);
    motor[2].init(3, 0, 10, 64);
    motor[3].init(4, 0, 1, 1);

    while (1) {
        key = key_scan(0);

        // if (key == KEY0_PRES && move1_state == MS1_IDLE) {
        //     displayMessage("Running Main Program");
        //     wait_time[0] = 20*60; //[s] 
        //     wait_time[1] = 30*60;
        //     move1_state = MS1_STEP1;       // Trigger state machine step 1
        // }
        if (key == KEY0_PRES && move2_state == MS2_IDLE) {
            // wait_time = {30, 60}; 
            displayMessage("Running Demo Program");
            //manual_set_1();
            wait_time[0] = 20; //[s] 
            wait_time[1] = 20;
            move2_state = MS2_STEP1;       // Trigger state machine step 1
        }
        if (key == KEY1_PRES) {
            displayMessage("Manual Control");
            manual_set_1();
        }
        if (key == KEY2_PRES) {
            displayMessage("STOP");
            // Stop all motors
            motor[0].tgt_position(0, 0);
            delay_ms(10);
            motor[1].tgt_position(0, 0);
            delay_ms(10);
            motor[2].tgt_position(0, 0);
            delay_ms(10);
            motor[3].tgt_position(0, 0);
        
            move1_state = MS1_IDLE;  // Interrupt state machine
            move2_state = MS2_IDLE;  // Interrupt state machine
        }

        process_move_set_1();          // Drive the state machine (non-blocking)
        process_move_set_2(); 
        // pollMotorStops();              // Auto-stop check for motors

        // Assume the maximum status name length of 6 characters, such as "WAIT2" or "STEP1"
        // Format it to fixed length 6 with a prefix; this ensures that if the status is "IDLE" (4 characters),
        // two trailing spaces are added to maintain consistent display width
        char stateBuf[16];
        if(move1_state == MS1_IDLE && move2_state != MS2_IDLE) {
            snprintf(stateBuf, sizeof(stateBuf), "MS2:%-6s", MoveState1Names[(int)move2_state]);
        } else {
            snprintf(stateBuf, sizeof(stateBuf), "MS1:%-6s", MoveState1Names[(int)move1_state]);
        }
        //snprintf(stateBuf, sizeof(stateBuf), "MS1:%-6s", MoveState1Names[(int)move1_state]);
        lcd_show_string(10, 10, 200, 16, 16, stateBuf, BLUE);

        // Periodically read RS485 and update the display
        if (++t >= 20) {
            t = 0;
            LED0_TOGGLE();
            cnt++;
            lcd_show_xnum(78, 130, cnt, 3, 16, 0x80, BLUE);
            for (uint8_t addr = 1; addr <= 4; ++addr) {
                Emm_V5_Read_Sys_Params(addr, S_VEL);
                Emm_V5_Read_Sys_Params(addr, S_CPOS);
                Emm_V5_Read_Sys_Params(addr, S_FLAG);
            }
        }

        Translate_received_data(rs485buf);

        HAL_Delay(1); // a very short delay to maintain the main loop frequency while remaining non-blocking
    }

    return 0;
}