#include "./c_bindings.h"
#include <cmath>
#include <string>

// --- State Machine Definition ---
enum MoveState1 {
    MS1_IDLE = 0,
    MS1_STEP1,
    MS1_WAIT1,
    MS1_STEP2,
    MS1_WAIT2,
    MS1_STEP3,
    MS1_DONE
};

static MoveState1 move1_state = MS1_IDLE;
static uint32_t move1_lastTick = 0;

const char* MoveState1Names[] = {
    "IDLE",
    "STEP1",
    "WAIT1",
    "STEP2",
    "WAIT2",
    "STEP3",
    "DONE"
};

// Define the message display area size
#define MSG_X      30
#define MSG_Y      150
#define MSG_W      500
#define MSG_H      100

void displayMessage(const char* msg) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%-20s", msg);
    lcd_show_string(MSG_X, MSG_Y, MSG_W, MSG_H, 16, buf, BLUE);
}

class Motor {
private:
    uint8_t addr;
    uint8_t set_dir;
    uint8_t dir;
    uint8_t redu_ratio;
    uint8_t acc;
    uint16_t vel;

public:
    int velocity;
    double tgt_degree;
    int32_t read_velocity;
    int32_t read_position_raw;
    double read_degree;
    bool reach_pos;
    std::string status;

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
        reach_pos = false;
        vel = 0;
        dir = set_dir;
        status = "STOP";
    }

    uint32_t tgt_position(double degree, uint16_t velocity_val = 1) {
        tgt_degree = degree;
        if ((velocity_val == 1 && vel == 0) || (velocity_val != 1)) {
            vel = velocity_val;
        }
        if (degree < 0) {
            dir = set_dir ? 0 : 1;
            degree = -degree;
            velocity = -vel;
        } else {
            dir = set_dir;
            velocity = vel;
        }
        uint32_t position = static_cast<uint32_t>(degree * 3200 * redu_ratio / 360);
        Emm_V5_Pos_Control(addr, dir, vel, acc, position, 0, 0);
        status = (velocity != 0) ? "RUN" : "STOP";
        return position;
    }

    void set_velocity(int velocity_val) {
        velocity = velocity_val;
        if (velocity < 0) {
            vel = static_cast<uint16_t>(-velocity);
            dir = set_dir ? 0 : 1;
        } else {
            vel = static_cast<uint16_t>(velocity);
            dir = set_dir;
        }
        status = (velocity != 0) ? "RUN" : "STOP";
    }

    void constant_rorate() {
        status = (vel != 0) ? "RUN" : "STOP";
        Emm_V5_Vel_Control(addr, dir, vel, acc, 0);
    }

    void constant_rorate(int velocity_val) {
        set_velocity(velocity_val);
        Emm_V5_Vel_Control(addr, dir, vel, acc, 0);
    }

    uint8_t get_reduction_ratio() const { return redu_ratio; }

    void displayStatus(int baseX, int baseY) const {
        char buf[32];
        int currentX = baseX;
        int currentY = baseY;
        const int offsetX = 10;

        // Motor label
        snprintf(buf, sizeof(buf), "M%02X:", addr);
        lcd_show_string(currentX, currentY, 60, 16, 16, buf, BLUE);
        currentX += 60 + offsetX * 2;

        // Status
        const char* st = status.c_str();
        uint16_t color = (status == "RUN") ? GREEN : RED;
        snprintf(buf, sizeof(buf), "%-4s", st);
        lcd_show_string(currentX, currentY, 60, 16, 16, buf, color);

        // Speed & direction
        currentY += 20;
        currentX = baseX;
        lcd_show_string(currentX, currentY, 60, 16, 16, "SPEED:", BLUE);
        currentX += 60 + offsetX;
        char displayBuf[16];
        if (velocity > 0) {
            const char* dirStr = (set_dir == 0) ? "CLW" : "CCLW";
            snprintf(displayBuf, sizeof(displayBuf), "%-4s%3d ", dirStr, velocity);
        } else if (velocity < 0) {
            const char* dirStr = (set_dir == 0) ? "CCLW" : "CLW";
            snprintf(displayBuf, sizeof(displayBuf), "%-4s%3d ", dirStr, -velocity);
        } else {
            snprintf(displayBuf, sizeof(displayBuf), "    0   ");
        }
        lcd_show_string(currentX, currentY, 70, 16, 16, displayBuf, BLUE);

        // Target angle
        currentY += 20;
        currentX = baseX;
        lcd_show_string(currentX, currentY, 100, 16, 16, "TARGET:", BLUE);
        currentX += 100 + offsetX;
        int tgt_int = static_cast<int>(tgt_degree);
        snprintf(buf, sizeof(buf), "%5d", tgt_int);
        lcd_show_string(currentX, currentY, 80, 16, 16, buf, BLUE);
    }
};

// --- Globals ---
Motor motor[5];
// static uint32_t motorStartTick[4] = {0};
// static bool motorMoving[4] = {false};
int wait_time[2] = {0};

void Translate_received_data(uint8_t* rs485buf) {
    uint8_t len;
    rs485_receive_data(rs485buf, &len);
    if (len == 0) return;
    if (len > 8) len = 8;

    uint8_t motor_addr = rs485buf[0];
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
        case 0x35:
            if (len >= 6) {
                uint8_t sign = rs485buf[2];
                uint16_t speed_raw = (rs485buf[3] << 8) | rs485buf[4];
                pm->read_velocity = (sign == 0x01) ? -static_cast<int32_t>(speed_raw) : speed_raw;
            }
            break;
        case 0x36:
            if (len >= 8) {
                uint8_t sign = rs485buf[2];
                uint32_t pos = (rs485buf[3] << 24) | (rs485buf[4] << 16) |
                               (rs485buf[5] << 8) | (rs485buf[6]);
                pm->read_position_raw = (sign == 0x01) ? -static_cast<int32_t>(pos) : pos;
                pm->read_degree = pm->read_position_raw * 360.0 / (3200.0 * pm->get_reduction_ratio());
            }
            break;
        case 0x3A:
            if (len >= 4) pm->reach_pos = (rs485buf[2] & 0x02);
            break;
        default:
            break;
    }

    int baseX = 30;
    for (uint8_t i = 0; i < 4; ++i) {
        int y = 210 + (i + 1) * 100;
        motor[i].displayStatus(baseX, y);
    }
}

void process_move_set_1(void) {
    uint32_t now = HAL_GetTick();
    switch (move1_state) {
        case MS1_IDLE:
            break;

        case MS1_STEP1:
            // 所有电机 RUN
            for (int i = 0; i < 4; ++i) motor[i].status = "RUN";
            motor[0].tgt_position(15, 30);
            delay_ms(10);
            motor[1].tgt_position(30, 20);
            delay_ms(10);
            motor[2].tgt_position(40, 10);
            delay_ms(10);
            motor[3].constant_rorate(10);

            move1_lastTick = now;
            move1_state = MS1_WAIT1;
            break;

        case MS1_WAIT1:
            // motor[3] RUN, 其他 STOP
            for (int i = 0; i < 3; ++i) motor[i].status = "STOP";
            motor[3].status = "RUN";
            if (now - move1_lastTick >= wait_time[0] * 1000) move1_state = MS1_STEP2;
            break;

        case MS1_STEP2:
            // 所有电机 RUN
            for (int i = 0; i < 4; ++i) motor[i].status = "RUN";
            motor[0].tgt_position(5, 6);
            delay_ms(10);
            motor[1].tgt_position(10, 4);
            delay_ms(10);
            motor[2].tgt_position(-20, 5);
            delay_ms(10);

            move1_state = MS1_WAIT2;
            break;

        case MS1_WAIT2:
            // motor[3] RUN, 其他 STOP
            for (int i = 0; i < 3; ++i) motor[i].status = "STOP";
            motor[3].status = "RUN";
            if (now - move1_lastTick >= wait_time[1] * 1000) move1_state = MS1_STEP3;
            break;

        case MS1_STEP3:
            // 所有电机 RUN
            for (int i = 0; i < 4; ++i) motor[i].status = "RUN";
            motor[0].tgt_position(-20, 30);
            motor[1].tgt_position(-40, 15);
            motor[2].tgt_position(-20, 5);
            motor[3].constant_rorate(0);
            move1_state = MS1_DONE;
            break;

        case MS1_DONE:
            for (int i = 0; i < 4; ++i) motor[i].status = "STOP";
            move1_state = MS1_IDLE;
            break;
    }
}

void move_set_2() {
    motor[2].tgt_position(10, 20);
}

int main(void) {
    HAL_Init();
    sys_stm32_clock_init(336, 8, 2, 7);
    delay_init(168);
    usart_init(115200);
    led_init();
    lcd_init();
    key_init();
    rs485_init(115200);

    lcd_show_string(30, 50, 200, 16, 16, "Senior Design:", RED);
    lcd_show_string(30, 70, 320, 16, 16, "Four-Axis Vacuum Stage", RED);
    lcd_show_string(30, 90, 320, 16, 16, "for Advanced Nano-Manufacturing", RED);

    motor[0].init(1, 1, 100, 128);
    motor[1].init(2, 1, 30, 128);
    motor[2].init(3, 0, 10, 64);
    motor[3].init(4, 0, 1, 1);

    uint8_t key;
    uint8_t t = 0, cnt = 0;
    uint8_t rs485buf[8];

    while (1) {
        key = key_scan(0);
        if (key == KEY0_PRES && move1_state == MS1_IDLE) {
            displayMessage("Running Main Program");
            wait_time[0] = 20;
            wait_time[1] = 30;
            move1_state = MS1_STEP1;
        }
        if (key == KEY1_PRES && move1_state == MS1_IDLE) {
            displayMessage("Running Program 2");
            move_set_2();
        }
        if (key == KEY2_PRES) {
            displayMessage("STOP");
            for (int i = 0; i < 4; ++i) {
                motor[i].tgt_position(0, 0);
                motor[i].status = "STOP";
            }
            move1_state = MS1_IDLE;
        }

        process_move_set_1();

        // 显示状态机状态
        char stateBuf[16];
        snprintf(stateBuf, sizeof(stateBuf), "MS1:%-6s", MoveState1Names[move1_state]);
        lcd_show_string(10, 10, 200, 16, 16, stateBuf, BLUE);

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
        HAL_Delay(1);
    }
    return 0;
}
