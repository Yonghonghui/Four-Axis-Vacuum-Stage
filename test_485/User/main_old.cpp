#include "./c_bindings.h"

// —— 状态机定义 ——
enum MoveState1 {
    MS1_IDLE = 0,
    MS1_STEP1,   // 初始动作
    MS1_WAIT1,   // 等待
    MS1_STEP2,   // 加速
    MS1_WAIT2,   // 等待
    MS1_STEP3,   // 降速并回到初始
    MS1_DONE     // 完成，可重置
};

static MoveState1 move1_state = MS1_IDLE;
static uint32_t move1_lastTick = 0;
// 等待时间（秒）
static const uint32_t wait_time1 = 30;
static const uint32_t wait_time2 = 60;

const char* MoveState1Names[] = {
    "IDLE",
    "STEP1",
    "WAIT1",
    "STEP2",
    "WAIT2",
    "STEP3",
    "DONE"
};

// 定义提示正在运行程序区域大小
#define MSG_X      30
#define MSG_Y      150
#define MSG_W      500
#define MSG_H      100


// 定义一个统一的输出函数，用固定宽度输出并自动补空格
void displayMessage(const char* msg) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%-20s", msg);
    lcd_show_string(MSG_X, MSG_Y, MSG_W, MSG_H, 16, buf, BLUE);
}

class Motor {
private:
    uint8_t addr;           // Motor address
    uint8_t set_dir;        // Set forward direction (0 or 1)
    uint8_t dir;            // Current direction bit sent to motor
    uint8_t redu_ratio;     // Reduction ratio of gearbox
    uint8_t acc;            // Acceleration parameter
    uint16_t vel;           // Velocity raw value

public:
    int velocity;           // User-defined speed (RPM)
    double tgt_degree;      // Target angle (degrees)
    int32_t read_velocity;  // Real-time velocity (RPM)
    int32_t read_position_raw; // Real-time raw position
    double read_degree;     // Real-time angle (degrees)
    bool reach_pos;         // Position reached flag
    int duration; // Duration for the motor to reach the target position
    const char* status;     // "RUN" or "STOP"

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
        status = "STOP";
    }

    uint32_t read_tgt_pos() const {
        return static_cast<uint32_t>(tgt_degree * (3200.0 * redu_ratio) / 360.0);
    }

    uint32_t tgt_position(double degree, int velocity_val = 1) {
        tgt_degree = degree;
        velocity = velocity_val;
        status = (velocity_val != 0) ? "RUN" : "STOP";
        if (degree < 0) {
            dir = set_dir ? 0 : 1;
            degree = -degree;
        } else {
            dir = set_dir;
        }
        uint32_t position = static_cast<uint32_t>(degree * (3200 * redu_ratio) / 360);
        duration = degree * redu_ratio *60000 /360 / vel; //[ms] // Calculate the duration to reach the target position
        if (velocity_val != 0) vel = static_cast<uint16_t>(velocity_val);
        Emm_V5_Pos_Control(addr, dir, vel, acc, position, 0, 0);
        return position;
    }

    void set_velocity(int velocity_val) {
        velocity = velocity_val;
        status = (velocity_val != 0) ? "RUN" : "STOP";
        if (velocity < 0) {
            vel = static_cast<uint16_t>(-velocity);
            dir = set_dir ? 0 : 1;
        } else {
            vel = static_cast<uint16_t>(velocity);
            dir = set_dir;
        }
    }

    void constant_rorate(int velocity_val) {
        set_velocity(velocity_val);
        Emm_V5_Vel_Control(addr, dir, vel, acc, 0);
    }

    uint8_t get_reduction_ratio() const { return redu_ratio; }

    void displayStatus(int baseX, int baseY) const {
        char buf[32];
        const int offsetX = 10;
        int currentY = baseY;
        int currentX = baseX;

        // Motor label
        snprintf(buf, sizeof(buf), "M%02X:", addr);
        lcd_show_string(currentX, currentY, 60, 16, 16, buf, BLUE);
        currentX += 60 + offsetX * 2;

        // Status
        uint16_t color = (status[0] == 'R') ? GREEN : RED;
        snprintf(buf, sizeof(buf), "%-4s", status);
        lcd_show_string(currentX, currentY, 60, 16, 16, buf, color);

        // SPEED & DIRECTION
        currentY += 20;
        currentX = baseX;
        lcd_show_string(currentX, currentY, 60, 16, 16, "SPEED:", BLUE);
        currentX += 60 + offsetX;
        char displayBuf[10];
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

        // TARGET DEGREE
        currentY += 20;
        currentX = baseX;
        lcd_show_string(currentX, currentY, 100, 16, 16, "TARGET:", BLUE);
        currentX += 100 + offsetX;
        int tgt_int = static_cast<int>(tgt_degree);
        snprintf(buf, sizeof(buf), "%5d", tgt_int);
        lcd_show_string(currentX, currentY, 80, 16, 16, buf, BLUE);
    }
};

// 全局四个电机
Motor motor1, motor2, motor3, motor4;
int wait_time[2] = {0, 0}; //[s] // Wait time for each motor
void Translate_received_data(uint8_t* rs485buf) {
    uint8_t len;
    rs485_receive_data(rs485buf, &len);
    if (len == 0) return;
    if (len > 8) len = 8;

    uint8_t motor_addr = rs485buf[0];
    uint8_t function_code = rs485buf[1];
    Motor* pm ;
    switch (motor_addr) {
        case 1: pm = &motor1; break;
        case 2: pm = &motor2; break;
        case 3: pm = &motor3; break;
        case 4: pm = &motor4; break;
        default: return;
    }

    switch (function_code) {
        case 0x35: // 速度反馈
            if (len >= 6) {
                uint8_t sign = rs485buf[2];
                uint16_t speed_raw = (rs485buf[3] << 8) | rs485buf[4];
                pm->read_velocity = (sign == 0x01) ? -static_cast<int32_t>(speed_raw) : speed_raw;
            }
            break;
        case 0x36: // 位置反馈
            if (len >= 8) {
                uint8_t sign = rs485buf[2];
                uint32_t pos = (rs485buf[3] << 24) | (rs485buf[4] << 16)
                               | (rs485buf[5] << 8)  | rs485buf[6];
                pm->read_position_raw = (sign == 0x01) ? -static_cast<int32_t>(pos) : pos;
                uint8_t rr = pm->get_reduction_ratio();
                pm->read_degree = pm->read_position_raw * 360.0 / (3200.0 * rr);
            }
            break;
        case 0x3A: // 状态标志
            if (len >= 4) {
                uint8_t status = rs485buf[2];
                pm->reach_pos = (status & 0x02) ? true : false;
            }
            break;
        default:
            break;
    }

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

void process_move_set_1(void) {
    uint32_t now = HAL_GetTick();
    switch (move1_state) {
        case MS1_IDLE:
            break;
        case MS1_STEP1:
            motor1.status = motor2.status = motor3.status = motor4.status = "RUN";
            motor1.tgt_position(15, 30);
            delay_ms(10);
            motor2.tgt_position(30, 20);
            delay_ms(10);
            motor3.tgt_position(40, 10);
            delay_ms(10);
            motor4.constant_rorate(10);
            delay_ms(10);
            move1_lastTick = now;
            move1_state = MS1_WAIT1;
            break;
        case MS1_WAIT1:
            if (now - move1_lastTick >= wait_time1 * 1000) move1_state = MS1_STEP2;
            motor1.status = motor2.status = motor3.status = "STOP";
            motor4.status = "RUN";
            break;
        case MS1_STEP2:

            motor1.tgt_position(5, 12);
            delay_ms(10);
            motor2.tgt_position(10, 8);
            delay_ms(10);
            motor3.tgt_position(-25, 5);
            delay_ms(10);
            motor4.constant_rorate(3);
            delay_ms(10);
            move1_lastTick = now;
            move1_state = MS1_WAIT2;
            motor1.status = motor2.status = motor3.status = motor4.status = "RUN";
            break;
        case MS1_WAIT2:
            if (now - move1_lastTick >= wait_time2 * 1000) move1_state = MS1_STEP3;
            motor1.status = motor2.status = motor3.status = "STOP";
            motor4.status = "RUN";
            break;
        case MS1_STEP3:
            motor1.tgt_position(-20, 40);
            delay_ms(10);
            motor2.tgt_position(-40, 15);
            delay_ms(10);
            motor3.tgt_position(-15, 3);
            delay_ms(10);
            motor4.constant_rorate(0);
            delay_ms(10);
            motor1.status = motor2.status = motor3.status = motor4.status = "RUN";
            move1_state = MS1_DONE;
            break;
        case MS1_DONE:
            move1_state = MS1_IDLE;
            motor1.status = motor2.status = motor3.status = motor4.status = "STOP";
            break;
    }
}

void move_set_2() {
    motor1.tgt_position(-3, 30);
    delay_ms(10);
    // motor2.tgt_position(-30, 30);
    // delay_ms(10);
    motor3.tgt_position(-50, 30);
    // 可添加其他动作
    // uint32_t now = HAL_GetTick();
    // motor4.constant_rorate(10);
    // delay_ms(10);
    
    // move1_lastTick = now;
    // if (now - move1_lastTick >= wait_time1*1000){
    //     motor4.constant_rorate(0);
        
    // }
}

int main(void) {
    uint8_t key, t = 0, cnt = 0;
    uint8_t rs485buf[8];
    char stateBuf[16];

    HAL_Init();
    sys_stm32_clock_init(336, 8, 2, 7);
    delay_init(168);
    usart_init(115200);
    led_init();
    lcd_init();
    key_init();
    rs485_init(115200);

    // 初始界面
    lcd_show_string(30, 50, 200, 16, 16, "Senior Design:", RED);
    lcd_show_string(30, 70, 320, 16, 16, "Four-Axis Vacuum Stage", RED);
    lcd_show_string(30, 90, 320, 16, 16, "for Advanced Nano-Manufacturing", RED);

    // 电机初始化
    motor1.init(1, 1, 100, 128);
    motor2.init(2, 1, 30, 128);
    motor3.init(3, 0, 10, 64);
    motor4.init(4, 0, 1, 1);

    while (1) {
        key = key_scan(0);
        if (key == KEY0_PRES && move1_state == MS1_IDLE) {
            displayMessage("Running Main Program");
            move1_state = MS1_STEP1;
        }
        if (key == KEY1_PRES && move1_state == MS1_IDLE) {
            displayMessage("Running Program 2");
            move_set_2();
        }
        if (key == KEY2_PRES) {
            displayMessage("STOP");
            motor1.tgt_position(0, 0);
            delay_ms(10);
            motor2.tgt_position(0, 0);
            delay_ms(10);
            motor3.tgt_position(0, 0);
            delay_ms(10);
            motor4.tgt_position(0, 0);
            move1_state = MS1_IDLE;
        }

        process_move_set_1();

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
