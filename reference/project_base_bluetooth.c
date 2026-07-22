/*
 * COSE421 Term Project - Maze Runner RC Car
 * Final autonomous Bluetooth build, comment-cleaned.
 *
 * Requirement mapping:
 * - Rear ultrasonic echo: EIC EXTINT0 interrupt on PA16, both edges.
 * - Accelerometer: SERCOM3 I2C interrupt read every 200 ms.
 * - Motor PWM: TCC0 outputs PA18/PA19.
 * - LiDAR: SERCOM2 UART packet polling, then front/right/left distance parsing.
 * - Bluetooth/console: SERCOM5 on PA20/PA21, 9600 bps when USE_BLUETOOTH_OUTPUT = 1.
 */

#pragma GCC target ("thumb")

#include <stdint.h>
#include "sam.h"

#define USE_BLUETOOTH_OUTPUT 1


#define PKG_HEADER          0x54
#define PKG_VER_LEN         0x2C
#define POINT_PER_PACK      12
#define LIDAR_PACKET_LEN    47

#define ANGLE_FULL          36000
#define ANGLE_FRONT         27000
#define ANGLE_RIGHT         18000
#define ANGLE_LEFT          0
#define ANGLE_TOLERANCE     500

#define UNKNOWN_CM          999
#define LIDAR_STALE_MS      1500
#define LIDAR_WAIT_STATUS_MS 1000


#define FILTER_SIZE         3


#define TRIG_PIN            17
#define ECHO_PIN            16
#define TRIG_MASK           (0x1ul << TRIG_PIN)

#define REAR_BLOCKED_CM     10
#define REAR_SAFE_CM        15
#define US_TIMEOUT_MS       40


#define LIS2DH_ADDR              0x18
#define LIS2DH_ADDR_WRITE        0x30
#define LIS2DH_ADDR_READ         0x31
#define LIS2DH_WHO_AM_I          0x0F
#define LIS2DH_CTRL_REG1         0x20
#define LIS2DH_CTRL_REG4         0x23
#define LIS2DH_TEMP_CFG_REG      0x1F
#define LIS2DH_OUT_X_L           0x28

#define ACCEL_PERIOD_MS          200
#define ACCEL_BUF_SIZE           6
#define ACCEL_CALIB_SAMPLES      10
#define ACCEL_DEADBAND_MG        40
#define ACCEL_SPEED_MAX_CMS      300

#define I2C_STATE_IDLE           0
#define I2C_STATE_SEND_REG       1
#define I2C_STATE_SEND_READ_ADDR 2
#define I2C_STATE_READING        3


#define LEFT_IN1_PIN        6
#define LEFT_IN2_PIN        7
#define RIGHT_IN1_PIN       8
#define RIGHT_IN2_PIN       9

#define LEFT_IN1_MASK       (0x1ul << LEFT_IN1_PIN)
#define LEFT_IN2_MASK       (0x1ul << LEFT_IN2_PIN)
#define RIGHT_IN1_MASK      (0x1ul << RIGHT_IN1_PIN)
#define RIGHT_IN2_MASK      (0x1ul << RIGHT_IN2_PIN)
#define MOTOR_DIR_MASK      (LEFT_IN1_MASK | LEFT_IN2_MASK | RIGHT_IN1_MASK | RIGHT_IN2_MASK)

#define PWM_PERIOD          1000


#define SPEED_STOP          0
#define SPEED_LOW           240
#define SPEED_MID           400
#define SPEED_HIGH          640
#define SPEED_MAX           700
#define SPEED_MIN           180
#define SPEED_STEP          50


#define TURN_SPEED          220
#define TURN_FORWARD_SPEED  410
#define TURN_BACKWARD_SPEED 240
#define PIVOT_SPEED         230
#define SOFT_TURN_DELTA     80
#define HARD_TURN_DELTA     90
#define TURN_DELTA          HARD_TURN_DELTA


#define REVERSE_KICK_SPEED      520
#define REVERSE_KICK_MS         100
#define ARC_OUTER_SPEED         460
#define ARC_INNER_SPEED         260
#define DEADEND_BACKUP_BASE_MS  500
#define DEADEND_BACKUP_STEP_MS  250
#define DEADEND_MAX_LEVEL       3
#define DEADEND_REPEAT_WINDOW_MS 4000
#define DEADEND_REPEAT_LIMIT     2


#define TURN_HISTORY_LEN        4
#define DEADEND_MEMORY_N        4


#define LEFT_PWM_PIN        18
#define RIGHT_PWM_PIN       19

#define DIR_STOP            0
#define DIR_FORWARD         1
#define DIR_BACKWARD        2

#define MODE_STOP           0
#define MODE_FORWARD        1
#define MODE_BACKWARD       2
#define MODE_LEFT           3
#define MODE_RIGHT          4


#define WALL_TARGET_CM      8
#define WALL_BAND_CM        3


#define WALL_KP             10
#define WALL_CORR_MAX       110
#define WALL_VALID_MIN_CM   5
#define WALL_VALID_MAX_CM   18
#define FRONT_HIGH_CM       30
#define FRONT_MID_CM        18
#define FRONT_LOW_CM        10
#define FRONT_TOOCLOSE_CM   8
#define FRONT_EMERGENCY_CM  6
#define SIDE_OPEN_CM        16
#define CORNER_FRONT_CM     10
#define DEADEND_SIDE_CM     10


#define HARD_TURN_MS        720
#define CORNER_TURN_MS      760
#define SIDE_OPEN_TURN_MS   660
#define PIVOT_180_MS        920
#define PIVOT_STRONG_MS     560
#define BACKUP_MS           450
#define STUCK_FRONT_CM      8
#define STUCK_TIME_MS       1200
#define STUCK_SPEED_CMS     4
#define STUCK_TURN_LIMIT    4
#define STUCK_TURN_WINDOW_MS 3000


#define FRONT_EMERGENCY_HOLD_MS 500
#define FRONT_CLEAR_CM          11


#define DEADEND_FRONT_CM        6
#define DEADEND_NOT_OPEN_CM     10
#define DECISION_COOLDOWN_MS    1000


#define FRONT_BLOCKED_CM        DEADEND_FRONT_CM
#define SIDE_BLOCKED_CM         8
#define SIDE_TURN_OPEN_CM       SIDE_OPEN_CM
#define CORNER_SIDE_WALL_CM     13
#define SIDE_CORNER_DIFF_CM     3


#define SIDE_DANGER_CM          7
#define SIDE_ESCAPE_MS          260


#define SIDE_CENTER_ACTIVE_CM   14
#define SIDE_CENTER_DEADBAND_CM 3
#define SIDE_CENTER_VALID_MAX_CM 40
#define SIDE_CENTER_KP          10
#define SIDE_CENTER_CORR_MAX    140


#define CONTACT_CM              4
#define CONTACT_BACKUP_MS       250


#define RIGHT_TURN_FRONT_MAX_CM 40


#define ULTRASONIC_PERIOD_MS 120
#define STATUS_PERIOD_MS     2000
#define TURN_COUNT_COOLDOWN_MS 1200
#define TURN_EXIT_FORWARD_MS   700


#define BAUD_9600           0xFB15
#define BAUD_115200         0xC504
#define BAUD_230400         0x8A09

#if USE_BLUETOOTH_OUTPUT
#define CONSOLE_BAUD        BAUD_9600
#else
#define CONSOLE_BAUD        BAUD_115200
#endif


typedef struct {
    uint16_t distance;
    uint8_t intensity;
} LiDARPointTypeDef;

typedef struct {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LiDARPointTypeDef point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} LiDARFrameTypeDef;


void initializeSystemFor48MHz(void);
void GCLK_setup(void);
void PORT_setup(void);
void USART_setup(void);
void EIC_setup(void);
void TCC0_setup(void);
void I2C_setup(void);
void LIS2DH_setup(void);
void LIS2DH_I2C_write(unsigned char reg_addr, unsigned char data);
unsigned char LIS2DH_I2C_read(unsigned char reg_addr);
void LIS2DH_start_accel_read_interrupt(void);
void SERCOM3_Handler(void);
void convert_accel_bytes_to_mg(unsigned char *buffer, short *x_mg, short *y_mg, short *z_mg);
void process_accelerometer_sample(void);
void reset_speed_estimator(void);
void console_print_speed_csv(void);
void RTC_setup_10us(void);

void SysTick_Handler(void);
unsigned int millis(void);

unsigned char read_lidar_packet(LiDARFrameTypeDef *packet);
void parse_lidar_packet(uint8_t *buf, LiDARFrameTypeDef *packet);
void update_lidar_distances(LiDARFrameTypeDef *packet);
void service_lidar_once(void);
void service_ultrasonic_periodic(void);
void service_accelerometer_periodic(void);
void service_background_tasks(void);
unsigned int angle_diff(unsigned int a, unsigned int b);
void expire_old_lidar_values(void);
unsigned int filter_push(unsigned int *buf, unsigned char *cnt, unsigned int sample);

void start_ultrasonic_measure(void);
void service_ultrasonic(void);
void EIC_Handler(void);

void control_vehicle_by_sensors(void);
unsigned char lidar_values_ready(void);
void do_maneuver(unsigned char mode, unsigned int speed, unsigned int duration_ms);
unsigned char rear_is_safe(void);
static unsigned int front_speed(unsigned int f);
unsigned char is_stuck(unsigned int f);
unsigned char front_emergency_persistent(unsigned int f);
unsigned char is_dead_end(unsigned int f, unsigned int r, unsigned int l,
                          unsigned char have_r, unsigned char have_l);
int is_front_blocked(void);
int is_left_blocked(void);
int is_right_blocked(void);
int choose_escape_direction(void);
unsigned char decision_locked(void);
void start_decision_cooldown(void);
void recover_from_stuck(void);
void avoid_front_obstacle(unsigned int r, unsigned int l, unsigned char have_r, unsigned char have_l);
unsigned char contact_escape(unsigned int f, unsigned int r, unsigned int l,
                             unsigned char have_r, unsigned char have_l);
void handle_dead_end(unsigned int r, unsigned int l, unsigned char have_r, unsigned char have_l);
void turn_and_advance(unsigned char mode, unsigned int duration_ms);
void pivot_toward_open(unsigned int r, unsigned int l, unsigned char have_r, unsigned char have_l);
void note_turn_dir(unsigned char mode);
void reset_stuck_state(void);


unsigned char arc_reverse(unsigned char toward_dir, unsigned int duration_ms);
void deadend_escape(unsigned int r, unsigned int l, unsigned char have_r, unsigned char have_l);
void push_turn_history(unsigned char mode);
unsigned int path_signature(void);
unsigned char side_signature(unsigned int r, unsigned int l,
                             unsigned char have_r, unsigned char have_l);
int recall_forbidden_dir(unsigned int sig, unsigned char sidesig);
void remember_dead_end(unsigned int sig, unsigned char sidesig, unsigned char chosen_dir);

void forward_RC(unsigned int speed);
void backward_RC(unsigned int speed);
void left_RC(unsigned int speed);
void right_RC(unsigned int speed);
void soft_left_RC(unsigned int speed);
void soft_right_RC(unsigned int speed);
void wall_follow_right_p(unsigned int spd, unsigned int r, unsigned char have_r);
void wall_follow_center_or_right_p(unsigned int spd, unsigned int r, unsigned int l,
                                   unsigned char have_r, unsigned char have_l);
void pivot_turn_180(void);
void stop_RC(void);
void update_vehicle(void);
void note_turn(void);
void set_left_dir(unsigned char direction);
void set_right_dir(unsigned char direction);
void set_motor_dir(unsigned char direction);
void set_motor_speed(unsigned int left_speed, unsigned int right_speed);

void process_user_command(void);
void cmd_speedup(void);
void cmd_speeddown(void);

void console_write_char(unsigned char c);
void console_print_cstr(const char *s);
void console_println_cstr(const char *s);
void console_print_unsigned_int(unsigned int data);
void console_print_int(int data);
void console_print_status(void);
void console_print_mode(void);
void console_print_menu(void);


volatile unsigned int g_ms = 0;


volatile unsigned int front_cm = UNKNOWN_CM;
volatile unsigned int right_cm = UNKNOWN_CM;
volatile unsigned int left_cm  = UNKNOWN_CM;
volatile unsigned int front_update_ms = 0;
volatile unsigned int right_update_ms = 0;
volatile unsigned int left_update_ms  = 0;


unsigned int front_filt[FILTER_SIZE];
unsigned int right_filt[FILTER_SIZE];
unsigned int left_filt[FILTER_SIZE];
unsigned char front_filt_cnt = 0;
unsigned char right_filt_cnt = 0;
unsigned char left_filt_cnt  = 0;


volatile unsigned int rear_ultrasonic_cm = UNKNOWN_CM;
volatile unsigned int echo_rise_ticks = 0;
volatile unsigned int echo_fall_ticks = 0;
volatile unsigned char echo_edge = 0;
volatile unsigned char echo_done = 0;
volatile unsigned int us_start_ms = 0;
volatile unsigned char us_in_flight = 0;


volatile unsigned char accel_i2c_busy = 0;
volatile unsigned char accel_i2c_done = 0;
volatile unsigned char accel_i2c_error = 0;
volatile unsigned char accel_i2c_state = I2C_STATE_IDLE;
volatile unsigned char accel_i2c_index = 0;
volatile unsigned char accel_i2c_buf[ACCEL_BUF_SIZE];

volatile short accel_x_mg = 0;
volatile short accel_y_mg = 0;
volatile short accel_z_mg = 0;
volatile int accel_x_offset_mg = 0;
volatile int accel_calib_sum = 0;
volatile unsigned int accel_calib_count = 0;
volatile int speed_cms = 0;
volatile unsigned int accel_sample_ms = 0;
volatile unsigned char accel_sample_ready = 0;
volatile unsigned char speed_csv_enabled = 1;
volatile unsigned char five_turns_reported = 0;

unsigned int turn_count = 0;
unsigned int last_turn_count_ms = 0;
unsigned int auto_forward_until_ms = 0;
unsigned int front_close_since_ms = 0;
unsigned int slow_stuck_since_ms = 0;
unsigned int last_turn_dir = MODE_STOP;
unsigned int same_dir_count = 0;
unsigned int same_dir_window_ms = 0;


unsigned char turn_history[TURN_HISTORY_LEN];
unsigned char turn_history_cnt = 0;
unsigned int de_sig[DEADEND_MEMORY_N];
unsigned char de_sidesig[DEADEND_MEMORY_N];
unsigned char de_last_dir[DEADEND_MEMORY_N];
unsigned char de_valid[DEADEND_MEMORY_N];
unsigned char de_head = 0;


unsigned int deadend_level = 0;
unsigned int last_deadend_ms = 0;
unsigned int deadend_repeat_count = 0;


unsigned int front_emergency_since_ms = 0;


unsigned int decision_lock_until_ms = 0;

unsigned int base_speed = SPEED_LOW;
unsigned char vehicle_mode = MODE_STOP;
unsigned char current_direction = DIR_STOP;
#define AUTO_START_ON_BOOT  1
unsigned char auto_drive_enabled = AUTO_START_ON_BOOT;


volatile unsigned int lidar_rx_byte_count = 0;
volatile unsigned int lidar_header_count = 0;
volatile unsigned int lidar_verlen_count = 0;
volatile unsigned int lidar_packet_count = 0;
volatile unsigned int lidar_last_start_angle = 0;
volatile unsigned int lidar_last_end_angle = 0;
volatile unsigned int lidar_front_hit_count = 0;
volatile unsigned int lidar_right_hit_count = 0;
volatile unsigned int lidar_left_hit_count = 0;


int main(void)
{
    unsigned int last_status_ms;

    SystemInit();
    initializeSystemFor48MHz();

    GCLK_setup();
    PORT_setup();
    USART_setup();


    console_println_cstr("DEBUG: UART ready before sensor init");

    RTC_setup_10us();
    EIC_setup();
    TCC0_setup();
    I2C_setup();

    console_println_cstr("DEBUG: before LIS2DH_setup");
    LIS2DH_setup();
    console_println_cstr("DEBUG: after LIS2DH_setup");

    SysTick_Config(48000);

    NVIC_EnableIRQ(EIC_IRQn);
    NVIC_SetPriority(EIC_IRQn, 2);
    NVIC_EnableIRQ(SERCOM3_IRQn);
    NVIC_SetPriority(SERCOM3_IRQn, 3);

    stop_RC();
    last_status_ms = millis();

#if USE_BLUETOOTH_OUTPUT
    console_println_cstr("Maze Runner start - Bluetooth HC-06 (SERCOM5, 9600)");
#else
    console_println_cstr("Maze Runner start - PC UART (SERCOM5, 115200)");
#endif
    console_println_cstr("LD19 LiDAR: SERCOM2, 230400 (polled, PA10/PA11 PMUX D)");
    console_println_cstr("LIS2DH12: SERCOM3 I2C, interrupt read every 200ms");
    console_println_cstr("Rear US: EIC echo interrupt. Motor PWM: TCC0 PA18/PA19.");
    console_println_cstr("AUTO mode is ON by default. Manual serial driving commands are disabled.");

    while (1) {

        service_background_tasks();


        if (auto_drive_enabled == 1) {
            control_vehicle_by_sensors();
        }


        process_user_command();


        if ((millis() - last_status_ms) >= STATUS_PERIOD_MS) {
            last_status_ms = millis();
            console_print_status();
        }
    }

    return 0;
}


void SysTick_Handler(void)
{
    g_ms++;
}

unsigned int millis(void)
{
    return g_ms;
}


static unsigned int micros_rtc(void)
{
    return RTC->MODE0.COUNT.reg / 8;
}


static unsigned int ticks_rtc(void)
{
    return RTC->MODE0.COUNT.reg;
}


unsigned char read_lidar_packet(LiDARFrameTypeDef *packet)
{
    static uint8_t buf[LIDAR_PACKET_LEN];
    static unsigned int count = 0;
    uint8_t rx_data;

    while (SERCOM2->USART.INTFLAG.bit.RXC == 1) {
        rx_data = (uint8_t)(SERCOM2->USART.DATA.reg & 0xFF);
        lidar_rx_byte_count++;

        if (rx_data == PKG_HEADER) {
            lidar_header_count++;
        }
        if (rx_data == PKG_VER_LEN) {
            lidar_verlen_count++;
        }

        if (count == 0) {
            if (rx_data == PKG_HEADER) {
                buf[0] = rx_data;
                count = 1;
            }
        }
        else if (count == 1) {
            if (rx_data == PKG_VER_LEN) {
                buf[1] = rx_data;
                count = 2;
            }
            else if (rx_data == PKG_HEADER) {
                buf[0] = rx_data;
                count = 1;
            }
            else {
                count = 0;
            }
        }
        else {
            buf[count] = rx_data;
            count++;

            if (count >= LIDAR_PACKET_LEN) {
                parse_lidar_packet(buf, packet);
                lidar_packet_count++;
                count = 0;
                return 1;
            }
        }
    }

    return 0;
}

void parse_lidar_packet(uint8_t *buf, LiDARFrameTypeDef *packet)
{
    unsigned int i;
    unsigned int idx;

    packet->header = buf[0];
    packet->ver_len = buf[1];
    packet->speed = (uint16_t)(buf[2] | (buf[3] << 8));
    packet->start_angle = (uint16_t)(buf[4] | (buf[5] << 8));

    for (i = 0; i < POINT_PER_PACK; i++) {
        idx = 6 + (i * 3);
        packet->point[i].distance = (uint16_t)(buf[idx] | (buf[idx + 1] << 8));
        packet->point[i].intensity = buf[idx + 2];
    }

    packet->end_angle = (uint16_t)(buf[42] | (buf[43] << 8));
    packet->timestamp = (uint16_t)(buf[44] | (buf[45] << 8));
    packet->crc8 = buf[46];
}

void service_lidar_once(void)
{


    LiDARFrameTypeDef packet;

    if (read_lidar_packet(&packet) == 1) {
        update_lidar_distances(&packet);
    }
    expire_old_lidar_values();
}

void service_ultrasonic_periodic(void)
{
    static unsigned int last_ultrasonic_ms = 0;

    if ((millis() - last_ultrasonic_ms) >= ULTRASONIC_PERIOD_MS) {
        last_ultrasonic_ms = millis();
        start_ultrasonic_measure();
    }
    service_ultrasonic();
}

void service_accelerometer_periodic(void)
{
    static unsigned int last_accel_ms = 0;

    if ((millis() - last_accel_ms) >= ACCEL_PERIOD_MS) {
        last_accel_ms = millis();
        LIS2DH_start_accel_read_interrupt();
    }
    process_accelerometer_sample();
}

void service_background_tasks(void)
{
    service_lidar_once();
    service_ultrasonic_periodic();
    service_accelerometer_periodic();
}


unsigned int filter_push(unsigned int *buf, unsigned char *cnt, unsigned int sample)
{
    unsigned int tmp[FILTER_SIZE];
    unsigned int i;
    unsigned int j;
    unsigned int n;
    unsigned int key;


    for (i = 1; i < FILTER_SIZE; i++) {
        buf[i - 1] = buf[i];
    }
    buf[FILTER_SIZE - 1] = sample;

    if (*cnt < FILTER_SIZE) {
        (*cnt)++;
    }

    n = *cnt;
    for (i = 0; i < n; i++) {
        tmp[i] = buf[FILTER_SIZE - n + i];
    }


    for (i = 1; i < n; i++) {
        key = tmp[i];
        j = i;
        while ((j > 0) && (tmp[j - 1] > key)) {
            tmp[j] = tmp[j - 1];
            j--;
        }
        tmp[j] = key;
    }

    return tmp[n / 2];
}

void update_lidar_distances(LiDARFrameTypeDef *packet)
{
    unsigned int i;
    unsigned int start_angle;
    unsigned int end_angle;
    unsigned int span;
    unsigned int angle;
    unsigned int distance_mm;
    unsigned int distance_cm;
    unsigned int now;

    unsigned int best_front = UNKNOWN_CM;
    unsigned int best_right = UNKNOWN_CM;
    unsigned int best_left = UNKNOWN_CM;

    start_angle = packet->start_angle;
    end_angle = packet->end_angle;
    lidar_last_start_angle = start_angle;
    lidar_last_end_angle = end_angle;
    now = millis();

    if (end_angle >= start_angle) {
        span = end_angle - start_angle;
    }
    else {
        span = (ANGLE_FULL - start_angle) + end_angle;
    }

    for (i = 0; i < POINT_PER_PACK; i++) {
        if (POINT_PER_PACK > 1) {
            angle = start_angle + ((span * i) / (POINT_PER_PACK - 1));
        }
        else {
            angle = start_angle;
        }

        if (angle >= ANGLE_FULL) {
            angle = angle - ANGLE_FULL;
        }

        distance_mm = packet->point[i].distance;

        if ((distance_mm >= 20) && (distance_mm <= 12000)) {
            distance_cm = distance_mm / 10;

            if (angle_diff(angle, ANGLE_FRONT) <= ANGLE_TOLERANCE) {
                if (distance_cm < best_front) best_front = distance_cm;
            }
            if (angle_diff(angle, ANGLE_RIGHT) <= ANGLE_TOLERANCE) {
                if (distance_cm < best_right) best_right = distance_cm;
            }
            if (angle_diff(angle, ANGLE_LEFT) <= ANGLE_TOLERANCE) {
                if (distance_cm < best_left) best_left = distance_cm;
            }
        }
    }


    if (best_front != UNKNOWN_CM) {
        lidar_front_hit_count++;
        front_cm = filter_push(front_filt, &front_filt_cnt, best_front);
        front_update_ms = now;
    }
    if (best_right != UNKNOWN_CM) {
        lidar_right_hit_count++;
        right_cm = filter_push(right_filt, &right_filt_cnt, best_right);
        right_update_ms = now;
    }
    if (best_left != UNKNOWN_CM) {
        lidar_left_hit_count++;
        left_cm = filter_push(left_filt, &left_filt_cnt, best_left);
        left_update_ms = now;
    }
}

unsigned int angle_diff(unsigned int a, unsigned int b)
{
    unsigned int diff;

    if (a >= b) {
        diff = a - b;
    }
    else {
        diff = b - a;
    }

    if (diff > (ANGLE_FULL / 2)) {
        diff = ANGLE_FULL - diff;
    }

    return diff;
}

void expire_old_lidar_values(void)
{
    unsigned int now;

    now = millis();

    if ((now - front_update_ms) > LIDAR_STALE_MS) {
        front_cm = UNKNOWN_CM;
        front_filt_cnt = 0;
    }
    if ((now - right_update_ms) > LIDAR_STALE_MS) {
        right_cm = UNKNOWN_CM;
        right_filt_cnt = 0;
    }
    if ((now - left_update_ms) > LIDAR_STALE_MS) {
        left_cm = UNKNOWN_CM;
        left_filt_cnt = 0;
    }
}


void LIS2DH_setup(void)
{
    unsigned char who_am_i;

    who_am_i = LIS2DH_I2C_read(LIS2DH_WHO_AM_I);

    console_print_cstr("LIS2DH12 WHO_AM_I=0x");
    console_write_char("0123456789ABCDEF"[(who_am_i >> 4) & 0x0F]);
    console_write_char("0123456789ABCDEF"[who_am_i & 0x0F]);
    console_write_char('\r');
    console_write_char('\n');


    LIS2DH_I2C_write(LIS2DH_CTRL_REG4, 0x80);

    LIS2DH_I2C_write(LIS2DH_TEMP_CFG_REG, 0xC0);

    LIS2DH_I2C_write(LIS2DH_CTRL_REG1, 0x57);

    reset_speed_estimator();
}


void LIS2DH_I2C_write(unsigned char reg_addr, unsigned char data)
{
    SERCOM3->I2CM.ADDR.bit.ADDR = LIS2DH_ADDR_WRITE;
    while ((SERCOM3->I2CM.INTFLAG.bit.MB != 1) || (SERCOM3->I2CM.STATUS.bit.RXNACK != 0)) ;

    SERCOM3->I2CM.DATA.bit.DATA = reg_addr;
    while ((SERCOM3->I2CM.INTFLAG.bit.MB != 1) || (SERCOM3->I2CM.STATUS.bit.RXNACK != 0)) ;

    SERCOM3->I2CM.DATA.bit.DATA = data;
    while ((SERCOM3->I2CM.INTFLAG.bit.MB != 1) || (SERCOM3->I2CM.STATUS.bit.RXNACK != 0)) ;

    SERCOM3->I2CM.CTRLB.bit.CMD = 3;
}

unsigned char LIS2DH_I2C_read(unsigned char reg_addr)
{
    unsigned char value;

    SERCOM3->I2CM.ADDR.bit.ADDR = LIS2DH_ADDR_WRITE;
    while ((SERCOM3->I2CM.INTFLAG.bit.MB != 1) || (SERCOM3->I2CM.STATUS.bit.RXNACK != 0)) ;

    SERCOM3->I2CM.DATA.bit.DATA = reg_addr;
    while ((SERCOM3->I2CM.INTFLAG.bit.MB != 1) || (SERCOM3->I2CM.STATUS.bit.RXNACK != 0)) ;

    SERCOM3->I2CM.ADDR.bit.ADDR = LIS2DH_ADDR_READ;
    while (SERCOM3->I2CM.INTFLAG.bit.SB != 1) ;


    SERCOM3->I2CM.CTRLB.bit.ACKACT = 1;
    value = (unsigned char)(SERCOM3->I2CM.DATA.bit.DATA & 0xFF);
    SERCOM3->I2CM.CTRLB.bit.CMD = 3;

    return value;
}

void LIS2DH_start_accel_read_interrupt(void)
{
    unsigned int i;

    if (accel_i2c_busy == 1) {
        return;
    }

    accel_i2c_busy = 1;
    accel_i2c_done = 0;
    accel_i2c_error = 0;
    accel_i2c_state = I2C_STATE_SEND_REG;
    accel_i2c_index = 0;

    for (i = 0; i < ACCEL_BUF_SIZE; i++) {
        accel_i2c_buf[i] = 0;
    }

    SERCOM3->I2CM.INTENCLR.bit.MB = 1;
    SERCOM3->I2CM.INTENCLR.bit.SB = 1;
    SERCOM3->I2CM.INTENCLR.bit.ERROR = 1;

    SERCOM3->I2CM.INTENSET.bit.MB = 1;
    SERCOM3->I2CM.INTENSET.bit.SB = 1;
    SERCOM3->I2CM.INTENSET.bit.ERROR = 1;


    SERCOM3->I2CM.ADDR.bit.ADDR = LIS2DH_ADDR_WRITE;
}


void SERCOM3_Handler(void)
{
    unsigned char data;

    if (SERCOM3->I2CM.INTFLAG.bit.ERROR == 1) {
        SERCOM3->I2CM.INTENCLR.bit.MB = 1;
        SERCOM3->I2CM.INTENCLR.bit.SB = 1;
        SERCOM3->I2CM.INTENCLR.bit.ERROR = 1;
        SERCOM3->I2CM.CTRLB.bit.CMD = 3;
        accel_i2c_error = 1;
        accel_i2c_busy = 0;
        accel_i2c_state = I2C_STATE_IDLE;
        return;
    }

    if (SERCOM3->I2CM.INTFLAG.bit.MB == 1) {
        if (SERCOM3->I2CM.STATUS.bit.RXNACK == 1) {
            SERCOM3->I2CM.INTENCLR.bit.MB = 1;
            SERCOM3->I2CM.INTENCLR.bit.SB = 1;
            SERCOM3->I2CM.INTENCLR.bit.ERROR = 1;
            SERCOM3->I2CM.CTRLB.bit.CMD = 3;
            accel_i2c_error = 1;
            accel_i2c_busy = 0;
            accel_i2c_state = I2C_STATE_IDLE;
            return;
        }

        if (accel_i2c_state == I2C_STATE_SEND_REG) {

            SERCOM3->I2CM.DATA.bit.DATA = LIS2DH_OUT_X_L | 0x80;
            accel_i2c_state = I2C_STATE_SEND_READ_ADDR;
        }
        else if (accel_i2c_state == I2C_STATE_SEND_READ_ADDR) {
            SERCOM3->I2CM.ADDR.bit.ADDR = LIS2DH_ADDR_READ;
            accel_i2c_state = I2C_STATE_READING;
        }
        else {
            SERCOM3->I2CM.INTENCLR.bit.MB = 1;
        }
    }

    if (SERCOM3->I2CM.INTFLAG.bit.SB == 1) {
        data = (unsigned char)(SERCOM3->I2CM.DATA.reg & 0xFF);

        if (accel_i2c_index < ACCEL_BUF_SIZE) {
            accel_i2c_buf[accel_i2c_index] = data;
            accel_i2c_index++;
        }

        if (accel_i2c_index >= ACCEL_BUF_SIZE) {
            SERCOM3->I2CM.CTRLB.bit.ACKACT = 1;
            SERCOM3->I2CM.CTRLB.bit.CMD = 3;

            SERCOM3->I2CM.INTENCLR.bit.MB = 1;
            SERCOM3->I2CM.INTENCLR.bit.SB = 1;
            SERCOM3->I2CM.INTENCLR.bit.ERROR = 1;

            accel_i2c_done = 1;
            accel_i2c_busy = 0;
            accel_i2c_state = I2C_STATE_IDLE;
        }
        else {
            SERCOM3->I2CM.CTRLB.bit.ACKACT = 0;
            SERCOM3->I2CM.CTRLB.bit.CMD = 2;
        }
    }
}

void convert_accel_bytes_to_mg(unsigned char *buffer, short *x_mg, short *y_mg, short *z_mg)
{
    short x;
    short y;
    short z;


    x = (short)((buffer[1] << 8) | buffer[0]) >> 6;
    y = (short)((buffer[3] << 8) | buffer[2]) >> 6;
    z = (short)((buffer[5] << 8) | buffer[4]) >> 6;

    *x_mg = (short)(x << 2);
    *y_mg = (short)(y << 2);
    *z_mg = (short)(z << 2);
}

void process_accelerometer_sample(void)
{
    unsigned char local_buf[ACCEL_BUF_SIZE];
    unsigned int i;
    short x_mg;
    short y_mg;
    short z_mg;
    int linear_x_mg;
    int dv_cms;

    if (accel_i2c_done == 0) {
        return;
    }

    for (i = 0; i < ACCEL_BUF_SIZE; i++) {
        local_buf[i] = (unsigned char)accel_i2c_buf[i];
    }
    accel_i2c_done = 0;

    convert_accel_bytes_to_mg(local_buf, &x_mg, &y_mg, &z_mg);

    accel_x_mg = x_mg;
    accel_y_mg = y_mg;
    accel_z_mg = z_mg;
    accel_sample_ms = millis();


    if (accel_calib_count < ACCEL_CALIB_SAMPLES) {
        accel_calib_sum += x_mg;
        accel_calib_count++;

        if (accel_calib_count == ACCEL_CALIB_SAMPLES) {
            accel_x_offset_mg = accel_calib_sum / ACCEL_CALIB_SAMPLES;
            console_print_cstr("Accel X offset calibrated: ");
            console_print_int(accel_x_offset_mg);
            console_println_cstr(" mg");
        }
        return;
    }

    linear_x_mg = x_mg - accel_x_offset_mg;

    if ((linear_x_mg < ACCEL_DEADBAND_MG) && (linear_x_mg > -ACCEL_DEADBAND_MG)) {
        linear_x_mg = 0;
    }


    dv_cms = (linear_x_mg * 981 * ACCEL_PERIOD_MS) / 1000000;
    speed_cms += dv_cms;

    if (speed_cms < 0) {
        speed_cms = 0;
    }
    else if (speed_cms > ACCEL_SPEED_MAX_CMS) {
        speed_cms = ACCEL_SPEED_MAX_CMS;
    }

    if ((linear_x_mg == 0) || (vehicle_mode == MODE_STOP)) {
        speed_cms = (speed_cms * 95) / 100;
    }

    accel_sample_ready = 1;

    if (speed_csv_enabled == 1) {
        console_print_speed_csv();
    }
}

void reset_speed_estimator(void)
{
    unsigned int i;

    accel_calib_sum = 0;
    accel_calib_count = 0;
    accel_x_offset_mg = 0;
    speed_cms = 0;
    turn_count = 0;
    last_turn_count_ms = 0;
    auto_forward_until_ms = 0;
    reset_stuck_state();
    five_turns_reported = 0;
    accel_sample_ready = 0;


    turn_history_cnt = 0;
    for (i = 0; i < TURN_HISTORY_LEN; i++) {
        turn_history[i] = MODE_STOP;
    }
    for (i = 0; i < DEADEND_MEMORY_N; i++) {
        de_sig[i] = 0;
        de_sidesig[i] = 0;
        de_last_dir[i] = MODE_STOP;
        de_valid[i] = 0;
    }
    de_head = 0;
    deadend_level = 0;
    last_deadend_ms = 0;
    deadend_repeat_count = 0;
}

void console_print_speed_csv(void)
{


    console_print_cstr("CSV,");
    console_print_unsigned_int(accel_sample_ms);
    console_write_char(',');
    console_print_int(speed_cms);
    console_write_char(',');
    console_print_unsigned_int(turn_count);
    console_write_char(',');
    console_print_unsigned_int(front_cm);
    console_write_char(',');
    console_print_unsigned_int(right_cm);
    console_write_char(',');
    console_print_unsigned_int(left_cm);
    console_write_char(',');
    console_print_unsigned_int(rear_ultrasonic_cm);
    console_write_char(',');
    console_print_int(accel_x_mg);
    console_write_char(',');
    console_print_int(accel_y_mg);
    console_write_char(',');
    console_print_int(accel_z_mg);
    console_write_char('\r');
    console_write_char('\n');

    if ((turn_count >= 5) && (five_turns_reported == 0)) {
        five_turns_reported = 1;
        console_println_cstr("# FIVE_TURNS_DONE: cut CSV log here and draw speed graph");
    }
}


static void ultrasonic_trigger_pulse(void)
{
    unsigned int t0;

    PORT->Group[0].OUTCLR.reg = TRIG_MASK;
    t0 = micros_rtc();
    while ((micros_rtc() - t0) < 2) ;

    PORT->Group[0].OUTSET.reg = TRIG_MASK;
    t0 = micros_rtc();
    while ((micros_rtc() - t0) < 10) ;

    PORT->Group[0].OUTCLR.reg = TRIG_MASK;
}


void start_ultrasonic_measure(void)
{
    if (us_in_flight == 1) {
        return;
    }

    echo_edge = 0;
    echo_done = 0;
    us_in_flight = 1;
    us_start_ms = millis();

    EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT0;
    ultrasonic_trigger_pulse();
}


void service_ultrasonic(void)
{
    unsigned int width_us;

    if (echo_done == 1) {
        unsigned int width_ticks;

        echo_done = 0;
        us_in_flight = 0;


        width_ticks = echo_fall_ticks - echo_rise_ticks;
        width_us = width_ticks / 8u;

        rear_ultrasonic_cm = width_us / 58;
        if (rear_ultrasonic_cm == 0) {
            rear_ultrasonic_cm = UNKNOWN_CM;
        }
        return;
    }


    if ((us_in_flight == 1) && ((millis() - us_start_ms) > US_TIMEOUT_MS)) {
        us_in_flight = 0;
        rear_ultrasonic_cm = UNKNOWN_CM;
    }
}


void EIC_Handler(void)
{
    unsigned int now_ticks;

    EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT0;
    now_ticks = ticks_rtc();

    if (echo_edge == 0) {
        echo_rise_ticks = now_ticks;
        echo_edge = 1;
    }
    else {
        echo_fall_ticks = now_ticks;
        echo_edge = 0;
        echo_done = 1;
    }
}


unsigned char rear_is_safe(void)
{

    if (rear_ultrasonic_cm == UNKNOWN_CM) {
        return 0;
    }
    return (rear_ultrasonic_cm >= REAR_SAFE_CM) ? 1 : 0;
}


void do_maneuver(unsigned char mode, unsigned int speed, unsigned int duration_ms)
{
    unsigned int t0 = millis();

    base_speed = speed;
    vehicle_mode = mode;

    if (mode == MODE_BACKWARD) {
        current_direction = DIR_BACKWARD;
    }
    else {
        current_direction = DIR_FORWARD;
    }
    update_vehicle();

    while ((millis() - t0) < duration_ms) {


        service_background_tasks();
        process_user_command();

        if (auto_drive_enabled == 0) {
            stop_RC();
            return;
        }


        if (mode == MODE_BACKWARD) {
            if ((rear_ultrasonic_cm != UNKNOWN_CM) &&
                (rear_ultrasonic_cm <= REAR_BLOCKED_CM)) {
                stop_RC();
                console_println_cstr("Rear unsafe: backward aborted");
                return;
            }
        }
    }

    stop_RC();
}


unsigned char arc_reverse(unsigned char toward_dir, unsigned int duration_ms)
{
    unsigned int t0;
    unsigned int left_spd;
    unsigned int right_spd;


    if ((rear_ultrasonic_cm != UNKNOWN_CM) &&
        (rear_ultrasonic_cm <= REAR_BLOCKED_CM)) {
        console_println_cstr("ARC: rear wall already close, skip reverse");
        stop_RC();
        return 0;
    }

    set_left_dir(DIR_BACKWARD);
    set_right_dir(DIR_BACKWARD);
    vehicle_mode = MODE_BACKWARD;
    current_direction = DIR_BACKWARD;


    set_motor_speed(REVERSE_KICK_SPEED, REVERSE_KICK_SPEED);
    t0 = millis();
    while ((millis() - t0) < REVERSE_KICK_MS) {
        service_background_tasks();
        process_user_command();
        if (auto_drive_enabled == 0) { stop_RC(); return 1; }
        if ((rear_ultrasonic_cm != UNKNOWN_CM) &&
            (rear_ultrasonic_cm <= REAR_BLOCKED_CM)) {
            stop_RC();
            console_println_cstr("ARC: rear close during kick -> stop");
            return 1;
        }
    }


    if (toward_dir == MODE_LEFT) {
        left_spd = ARC_INNER_SPEED;
        right_spd = ARC_OUTER_SPEED;
        note_turn_dir(MODE_LEFT);
        console_println_cstr("ARC REVERSE: nose -> LEFT");
    }
    else {
        left_spd = ARC_OUTER_SPEED;
        right_spd = ARC_INNER_SPEED;
        note_turn_dir(MODE_RIGHT);
        console_println_cstr("ARC REVERSE: nose -> RIGHT");
    }
    set_motor_speed(left_spd, right_spd);

    t0 = millis();
    while ((millis() - t0) < duration_ms) {
        service_background_tasks();
        process_user_command();
        if (auto_drive_enabled == 0) { stop_RC(); return 1; }
        if ((rear_ultrasonic_cm != UNKNOWN_CM) &&
            (rear_ultrasonic_cm <= REAR_BLOCKED_CM)) {
            stop_RC();
            console_println_cstr("ARC: rear wall close -> stop");
            return 1;
        }
    }
    stop_RC();
    return 1;
}


void deadend_escape(unsigned int r, unsigned int l, unsigned char have_r, unsigned char have_l)
{
    unsigned int now = millis();
    unsigned int sig;
    unsigned char sidesig;
    unsigned char toward;
    unsigned int backup_ms;
    int forbidden;

    if ((last_deadend_ms != 0) &&
        ((now - last_deadend_ms) <= DEADEND_REPEAT_WINDOW_MS)) {
        deadend_repeat_count++;
        if (deadend_repeat_count >= DEADEND_REPEAT_LIMIT) {
            if (deadend_level < DEADEND_MAX_LEVEL) {
                deadend_level++;
            }
            deadend_repeat_count = 0;
        }
    }
    else {
        deadend_level = 0;
        deadend_repeat_count = 0;
    }
    last_deadend_ms = now;

    console_print_cstr("DEAD-END ARC escape, level=");
    console_print_unsigned_int(deadend_level);
    console_write_char('\r');
    console_write_char('\n');

    stop_RC();

    sig = path_signature();
    sidesig = side_signature(r, l, have_r, have_l);
    forbidden = recall_forbidden_dir(sig, sidesig);

    if (have_r && have_l) {
        toward = (r >= l) ? MODE_RIGHT : MODE_LEFT;
    }
    else if (have_r) {
        toward = MODE_RIGHT;
    }
    else if (have_l) {
        toward = MODE_LEFT;
    }
    else {
        toward = (last_turn_dir == MODE_RIGHT) ? MODE_LEFT : MODE_RIGHT;
    }

    if (((forbidden == MODE_LEFT) || (forbidden == MODE_RIGHT)) &&
        (toward == forbidden)) {
        toward = (toward == MODE_LEFT) ? MODE_RIGHT : MODE_LEFT;
        console_println_cstr("MEMORY: flip repeated dead-end exit");
    }

    if (deadend_level >= DEADEND_MAX_LEVEL) {
        toward = (toward == MODE_LEFT) ? MODE_RIGHT : MODE_LEFT;
        console_println_cstr("ESCALATE: opposite dead-end escape");
    }

    remember_dead_end(sig, sidesig, toward);
    backup_ms = DEADEND_BACKUP_BASE_MS + (deadend_level * DEADEND_BACKUP_STEP_MS);

    if (auto_drive_enabled == 1) {


        unsigned char reversed = arc_reverse(toward, backup_ms);

        if (reversed == 0) {
            console_println_cstr("DEAD-END: rear genuinely blocked -> pivot fallback");
            if (have_r || have_l) {
                pivot_toward_open(r, l, have_r, have_l);
            }
            else {
                pivot_turn_180();
            }
        }
    }

    if (auto_drive_enabled == 1) {
        auto_forward_until_ms = millis() + TURN_EXIT_FORWARD_MS;
        start_decision_cooldown();
    }

    reset_stuck_state();
}

void pivot_turn_180(void)
{


    base_speed = PIVOT_SPEED;
    vehicle_mode = MODE_LEFT;
    current_direction = DIR_FORWARD;

    set_left_dir(DIR_BACKWARD);
    set_right_dir(DIR_FORWARD);
    set_motor_speed(PIVOT_SPEED, PIVOT_SPEED);
    note_turn_dir(MODE_LEFT);

    {
        unsigned int t0 = millis();
        while ((millis() - t0) < PIVOT_180_MS) {
            service_background_tasks();
            process_user_command();
            if (auto_drive_enabled == 0) {
                stop_RC();
                return;
            }
        }
    }
    stop_RC();
}


void pivot_toward_open(unsigned int r, unsigned int l, unsigned char have_r, unsigned char have_l)
{
    unsigned char go_right;


    if ((have_r == 1) && (have_l == 1)) {
        go_right = (r >= l) ? 1 : 0;
    }
    else if (have_r == 1) {
        go_right = 1;
    }
    else if (have_l == 1) {
        go_right = 0;
    }
    else {
        go_right = 0;
    }

    base_speed = PIVOT_SPEED;
    current_direction = DIR_FORWARD;

    if (go_right == 1) {

        vehicle_mode = MODE_RIGHT;
        set_left_dir(DIR_FORWARD);
        set_right_dir(DIR_BACKWARD);
        note_turn_dir(MODE_RIGHT);
    }
    else {

        vehicle_mode = MODE_LEFT;
        set_left_dir(DIR_BACKWARD);
        set_right_dir(DIR_FORWARD);
        note_turn_dir(MODE_LEFT);
    }
    set_motor_speed(PIVOT_SPEED, PIVOT_SPEED);

    {
        unsigned int t0 = millis();
        while ((millis() - t0) < PIVOT_STRONG_MS) {
            service_background_tasks();
            process_user_command();
            if (auto_drive_enabled == 0) {
                stop_RC();
                return;
            }
        }
    }
    stop_RC();
}


unsigned char lidar_values_ready(void)
{


    if ((front_cm == UNKNOWN_CM) ||
        (right_cm == UNKNOWN_CM) ||
        (left_cm == UNKNOWN_CM)) {
        return 0;
    }
    return 1;
}


static unsigned int front_speed(unsigned int f)
{
    if (f >= FRONT_HIGH_CM) {
        return SPEED_HIGH;
    }
    if (f >= FRONT_MID_CM) {
        return SPEED_MID;
    }
    return SPEED_LOW;
}

void reset_stuck_state(void)
{
    front_close_since_ms = 0;
    slow_stuck_since_ms = 0;
    last_turn_dir = MODE_STOP;
    same_dir_count = 0;
    same_dir_window_ms = 0;
    front_emergency_since_ms = 0;


}


unsigned char front_emergency_persistent(unsigned int f)
{
    unsigned int now = millis();

    if (f <= FRONT_EMERGENCY_CM) {
        if (front_emergency_since_ms == 0) {
            front_emergency_since_ms = now;
        }
        if ((now - front_emergency_since_ms) >= FRONT_EMERGENCY_HOLD_MS) {
            return 1;
        }
    }
    else if (f >= FRONT_CLEAR_CM) {
        front_emergency_since_ms = 0;
    }


    return 0;
}


unsigned char is_dead_end(unsigned int f, unsigned int r, unsigned int l,
                          unsigned char have_r, unsigned char have_l)
{
    if (f > DEADEND_FRONT_CM) {
        return 0;
    }


    if ((have_r == 0) || (have_l == 0)) {
        return 0;
    }

    return ((r <= DEADEND_NOT_OPEN_CM) && (l <= DEADEND_NOT_OPEN_CM)) ? 1 : 0;
}


int is_front_blocked(void)
{
    if (front_cm == UNKNOWN_CM) {
        return 1;
    }
    return (front_cm <= FRONT_BLOCKED_CM) ? 1 : 0;
}


int is_left_blocked(void)
{
    if (left_cm == UNKNOWN_CM) {
        return 0;
    }
    return (left_cm <= SIDE_BLOCKED_CM) ? 1 : 0;
}

int is_right_blocked(void)
{
    if (right_cm == UNKNOWN_CM) {
        return 0;
    }
    return (right_cm <= SIDE_BLOCKED_CM) ? 1 : 0;
}


int choose_escape_direction(void)
{
    unsigned char have_l = (left_cm != UNKNOWN_CM) ? 1 : 0;
    unsigned char have_r = (right_cm != UNKNOWN_CM) ? 1 : 0;


    unsigned char left_wall  = (have_l && (left_cm  <= CORNER_SIDE_WALL_CM)) ? 1 : 0;
    unsigned char right_wall = (have_r && (right_cm <= CORNER_SIDE_WALL_CM)) ? 1 : 0;
    unsigned char left_open  = (have_l && (left_cm  >= SIDE_TURN_OPEN_CM)) ? 1 : 0;
    unsigned char right_open = (have_r && (right_cm >= SIDE_TURN_OPEN_CM)) ? 1 : 0;


    if ((front_cm != UNKNOWN_CM) && (front_cm <= CORNER_FRONT_CM)) {
        if (right_wall && !left_wall) {
            return MODE_LEFT;
        }
        if (left_wall && !right_wall) {
            return MODE_RIGHT;
        }


        if (have_l && have_r) {
            if ((right_cm + SIDE_CORNER_DIFF_CM) < left_cm) {
                return MODE_LEFT;
            }
            if ((left_cm + SIDE_CORNER_DIFF_CM) < right_cm) {
                return MODE_RIGHT;
            }
        }


        if (right_wall && !have_l) {
            return MODE_LEFT;
        }
        if (left_wall && !have_r) {
            return MODE_RIGHT;
        }
    }


    if (right_open && !left_open) {
        return MODE_RIGHT;
    }
    if (left_open && !right_open) {
        return MODE_LEFT;
    }


    if (left_open && right_open) {
        if (left_cm > right_cm) {
            return MODE_LEFT;
        }
        return MODE_RIGHT;
    }


    if (right_wall && !left_wall) {
        return MODE_LEFT;
    }
    if (left_wall && !right_wall) {
        return MODE_RIGHT;
    }


    if (left_wall && right_wall) {
        return MODE_BACKWARD;
    }


    if (last_turn_dir == MODE_RIGHT) {
        return MODE_LEFT;
    }
    if (last_turn_dir == MODE_LEFT) {
        return MODE_RIGHT;
    }


    return MODE_LEFT;
}


unsigned char decision_locked(void)
{
    if (decision_lock_until_ms == 0) {
        return 0;
    }
    return (millis() < decision_lock_until_ms) ? 1 : 0;
}

void start_decision_cooldown(void)
{
    decision_lock_until_ms = millis() + DECISION_COOLDOWN_MS;
}

unsigned char is_stuck(unsigned int f)
{
    unsigned int now = millis();


    if (f <= STUCK_FRONT_CM) {
        if (front_close_since_ms == 0) {
            front_close_since_ms = now;
        }
        else if ((now - front_close_since_ms) >= STUCK_TIME_MS) {
            return 1;
        }
    }
    else {
        front_close_since_ms = 0;
    }


    if ((same_dir_count >= STUCK_TURN_LIMIT) &&
        ((now - same_dir_window_ms) <= STUCK_TURN_WINDOW_MS) &&
        (f <= CORNER_FRONT_CM)) {
        return 1;
    }


    if ((vehicle_mode != MODE_STOP) &&
        (speed_cms < STUCK_SPEED_CMS) &&
        (f <= (STUCK_FRONT_CM + 4))) {
        if (slow_stuck_since_ms == 0) {
            slow_stuck_since_ms = now;
        }
        else if ((now - slow_stuck_since_ms) >= STUCK_TIME_MS) {
            return 1;
        }
    }
    else {
        slow_stuck_since_ms = 0;
    }

    return 0;
}

void turn_and_advance(unsigned char mode, unsigned int duration_ms)
{
    do_maneuver(mode, SPEED_LOW, duration_ms);

    if (auto_drive_enabled == 1) {
        note_turn_dir(mode);


        auto_forward_until_ms = millis() + TURN_EXIT_FORWARD_MS;
        start_decision_cooldown();
    }
}

void handle_dead_end(unsigned int r, unsigned int l, unsigned char have_r, unsigned char have_l)
{

    deadend_escape(r, l, have_r, have_l);
}


void avoid_front_obstacle(unsigned int r, unsigned int l, unsigned char have_r, unsigned char have_l)
{
    int dir;


    if (decision_locked() && !(is_left_blocked() && is_right_blocked())) {
        if (last_turn_dir == MODE_LEFT) {
            do_maneuver(MODE_LEFT, SPEED_LOW, HARD_TURN_MS / 2);
            auto_forward_until_ms = millis() + TURN_EXIT_FORWARD_MS;
            return;
        }
        if (last_turn_dir == MODE_RIGHT) {
            do_maneuver(MODE_RIGHT, SPEED_LOW, HARD_TURN_MS / 2);
            auto_forward_until_ms = millis() + TURN_EXIT_FORWARD_MS;
            return;
        }
        forward_RC(SPEED_LOW);
        return;
    }

    dir = choose_escape_direction();


    if (dir == MODE_LEFT) {
        turn_and_advance(MODE_LEFT, CORNER_TURN_MS);
        return;
    }
    if (dir == MODE_RIGHT) {
        turn_and_advance(MODE_RIGHT, CORNER_TURN_MS);
        return;
    }


    handle_dead_end(r, l, have_r, have_l);
}


unsigned char contact_escape(unsigned int f, unsigned int r, unsigned int l,
                             unsigned char have_r, unsigned char have_l)
{
    unsigned char front_contact;
    unsigned char right_contact;
    unsigned char left_contact;
    unsigned char dir;

    front_contact = (f <= CONTACT_CM) ? 1 : 0;
    right_contact = ((have_r == 1) && (r <= CONTACT_CM)) ? 1 : 0;
    left_contact  = ((have_l == 1) && (l <= CONTACT_CM)) ? 1 : 0;

    if ((front_contact == 0) && (right_contact == 0) && (left_contact == 0)) {
        return 0;
    }

    console_println_cstr("CONTACT ESCAPE");
    stop_RC();


    if (rear_is_safe()) {
        do_maneuver(MODE_BACKWARD, SPEED_LOW, CONTACT_BACKUP_MS);
        stop_RC();
    }
    else {
        console_println_cstr("CONTACT: rear not safe, skip backup");
    }


    if ((right_contact == 1) && (left_contact == 0)) {
        dir = MODE_LEFT;
    }
    else if ((left_contact == 1) && (right_contact == 0)) {
        dir = MODE_RIGHT;
    }
    else if ((have_r == 1) && (have_l == 1)) {
        if (l > r) {
            dir = MODE_LEFT;
        }
        else if (r > l) {
            dir = MODE_RIGHT;
        }
        else if (last_turn_dir == MODE_RIGHT) {
            dir = MODE_LEFT;
        }
        else {
            dir = MODE_RIGHT;
        }
    }
    else if (have_l == 1) {
        dir = MODE_LEFT;
    }
    else if (have_r == 1) {
        dir = MODE_RIGHT;
    }
    else if (last_turn_dir == MODE_RIGHT) {
        dir = MODE_LEFT;
    }
    else {
        dir = MODE_RIGHT;
    }

    if (dir == MODE_LEFT) {
        console_println_cstr("CONTACT: turn LEFT");
        turn_and_advance(MODE_LEFT, CORNER_TURN_MS);
    }
    else {
        console_println_cstr("CONTACT: turn RIGHT");
        turn_and_advance(MODE_RIGHT, CORNER_TURN_MS);
    }

    return 1;
}


void control_vehicle_by_sensors(void)
{
    unsigned int f;
    unsigned int r;
    unsigned int l;
    unsigned int now = millis();
    unsigned int spd;

    unsigned char have_f;
    unsigned char have_r;
    unsigned char have_l;
    unsigned char right_open;
    unsigned char left_open;

    static unsigned int last_wait_msg_ms = 0;

    have_f = (front_cm != UNKNOWN_CM) ? 1 : 0;
    have_r = (right_cm != UNKNOWN_CM) ? 1 : 0;
    have_l = (left_cm != UNKNOWN_CM) ? 1 : 0;


    if (have_f == 0) {
        if ((millis() - last_wait_msg_ms) >= LIDAR_WAIT_STATUS_MS) {
            last_wait_msg_ms = millis();
            console_println_cstr("AUTO scan: LiDAR front UNKNOWN, rotating to reacquire");
        }

        if (last_turn_dir == MODE_LEFT) {
            left_RC(TURN_SPEED);
        }
        else {
            right_RC(TURN_SPEED);
        }
        return;
    }

    f = front_cm;
    r = have_r ? right_cm : WALL_TARGET_CM;
    l = have_l ? left_cm : 0;

    right_open = ((have_r == 1) && (r >= SIDE_OPEN_CM)) ? 1 : 0;
    left_open = ((have_l == 1) && (l >= SIDE_OPEN_CM)) ? 1 : 0;


    if (contact_escape(f, r, l, have_r, have_l) == 1) {
        return;
    }


    if ((f <= CORNER_FRONT_CM) && !decision_locked()) {


        unsigned char corner_left_wall;
        unsigned char corner_right_wall;

        corner_left_wall = ((have_l == 1) && (l <= CORNER_SIDE_WALL_CM)) ? 1 : 0;
        corner_right_wall = ((have_r == 1) && (r <= CORNER_SIDE_WALL_CM)) ? 1 : 0;

        if ((corner_right_wall == 1) && (corner_left_wall == 0)) {
            console_println_cstr("L-CORNER: right wall -> LEFT turn");
            turn_and_advance(MODE_LEFT, CORNER_TURN_MS);
            return;
        }
        if ((corner_left_wall == 1) && (corner_right_wall == 0)) {
            console_println_cstr("L-CORNER: left wall -> RIGHT turn");
            turn_and_advance(MODE_RIGHT, CORNER_TURN_MS);
            return;
        }


        if ((have_r == 1) && (have_l == 1)) {
            if ((r + SIDE_CORNER_DIFF_CM) < l) {
                console_println_cstr("L-CORNER: left side wider -> LEFT turn");
                turn_and_advance(MODE_LEFT, CORNER_TURN_MS);
                return;
            }
            if ((l + SIDE_CORNER_DIFF_CM) < r) {
                console_println_cstr("L-CORNER: right side wider -> RIGHT turn");
                turn_and_advance(MODE_RIGHT, CORNER_TURN_MS);
                return;
            }
        }
    }


    if (auto_forward_until_ms != 0) {
        if (now < auto_forward_until_ms) {
            if (f <= FRONT_TOOCLOSE_CM) {
                auto_forward_until_ms = 0;
                avoid_front_obstacle(r, l, have_r, have_l);
            }
            else {


                wall_follow_center_or_right_p(SPEED_LOW, r, l, have_r, have_l);
            }
            return;
        }
        auto_forward_until_ms = 0;
    }


    if (is_front_blocked()) {
        int side_open = (!is_left_blocked()) || (!is_right_blocked());

        if (side_open) {
            avoid_front_obstacle(r, l, have_r, have_l);
            return;
        }


    }


    if ((front_emergency_persistent(f) == 1) &&
        is_left_blocked() && is_right_blocked()) {
        recover_from_stuck();
        return;
    }

    if (is_stuck(f) == 1) {
        recover_from_stuck();
        return;
    }


    if (is_dead_end(f, r, l, have_r, have_l) == 1) {
        handle_dead_end(r, l, have_r, have_l);
        return;
    }


    if (f <= FRONT_TOOCLOSE_CM) {
        avoid_front_obstacle(r, l, have_r, have_l);
        return;
    }


    if ((have_r == 1) && (r <= SIDE_DANGER_CM) && (f > FRONT_TOOCLOSE_CM)) {
        console_println_cstr("SIDE AVOID: right too close -> LEFT");
        do_maneuver(MODE_LEFT, SPEED_LOW, SIDE_ESCAPE_MS);
        note_turn_dir(MODE_LEFT);
        auto_forward_until_ms = millis() + TURN_EXIT_FORWARD_MS;
        start_decision_cooldown();
        return;
    }

    if ((have_l == 1) && (l <= SIDE_DANGER_CM) && (f > FRONT_TOOCLOSE_CM)) {
        console_println_cstr("SIDE AVOID: left too close -> RIGHT");
        do_maneuver(MODE_RIGHT, SPEED_LOW, SIDE_ESCAPE_MS);
        note_turn_dir(MODE_RIGHT);
        auto_forward_until_ms = millis() + TURN_EXIT_FORWARD_MS;
        start_decision_cooldown();
        return;
    }


    if (decision_locked()) {
        spd = front_speed(f);
        forward_RC(spd);
        return;
    }


    if ((right_open == 1) && (f <= RIGHT_TURN_FRONT_MAX_CM) && !decision_locked()) {
        turn_and_advance(MODE_RIGHT, SIDE_OPEN_TURN_MS);
        return;
    }


    if ((f <= FRONT_LOW_CM) && (left_open == 1)) {
        turn_and_advance(MODE_LEFT, CORNER_TURN_MS);
        return;
    }


    spd = front_speed(f);
    wall_follow_center_or_right_p(spd, r, l, have_r, have_l);
}

void recover_from_stuck(void)
{
    console_println_cstr("STUCK/EMERGENCY -> recovery");
    stop_RC();


    if (auto_drive_enabled == 1) {
        int dir = choose_escape_direction();
        if (dir == MODE_LEFT) {
            turn_and_advance(MODE_LEFT, CORNER_TURN_MS);
            reset_stuck_state();
            return;
        }
        if (dir == MODE_RIGHT) {
            turn_and_advance(MODE_RIGHT, CORNER_TURN_MS);
            reset_stuck_state();
            return;
        }
    }


    if (auto_drive_enabled == 1) {
        unsigned char have_r = (right_cm != UNKNOWN_CM) ? 1 : 0;
        unsigned char have_l = (left_cm != UNKNOWN_CM) ? 1 : 0;
        deadend_escape(right_cm, left_cm, have_r, have_l);
    }

    reset_stuck_state();
}


void note_turn(void)
{
    unsigned int now = millis();

    if ((turn_count == 0) || ((now - last_turn_count_ms) >= TURN_COUNT_COOLDOWN_MS)) {
        turn_count++;
        last_turn_count_ms = now;
    }
}

void note_turn_dir(unsigned char mode)
{
    unsigned int now = millis();

    note_turn();
    if ((mode == MODE_LEFT) || (mode == MODE_RIGHT)) {
        push_turn_history(mode);
    }

    if ((mode == last_turn_dir) &&
        ((now - same_dir_window_ms) <= STUCK_TURN_WINDOW_MS)) {
        same_dir_count++;
    }
    else {
        same_dir_count = 1;
        same_dir_window_ms = now;
    }

    last_turn_dir = mode;
}


void push_turn_history(unsigned char mode)
{
    unsigned int i;

    for (i = 1; i < TURN_HISTORY_LEN; i++) {
        turn_history[i - 1] = turn_history[i];
    }
    turn_history[TURN_HISTORY_LEN - 1] = mode;
    if (turn_history_cnt < TURN_HISTORY_LEN) {
        turn_history_cnt++;
    }
}


unsigned int path_signature(void)
{
    unsigned int sig = 0;
    unsigned int i;

    for (i = 0; i < TURN_HISTORY_LEN; i++) {
        unsigned int code;
        if (turn_history[i] == MODE_LEFT)       code = 1;
        else if (turn_history[i] == MODE_RIGHT) code = 2;
        else                                    code = 0;
        sig = (sig << 2) | code;
    }
    return sig;
}


unsigned char side_signature(unsigned int r, unsigned int l,
                             unsigned char have_r, unsigned char have_l)
{
    unsigned char rb;
    unsigned char lb;

    if (have_r == 0)              rb = 0;
    else if (r <= SIDE_BLOCKED_CM) rb = 1;
    else if (r < SIDE_OPEN_CM)     rb = 2;
    else                           rb = 3;

    if (have_l == 0)              lb = 0;
    else if (l <= SIDE_BLOCKED_CM) lb = 1;
    else if (l < SIDE_OPEN_CM)     lb = 2;
    else                           lb = 3;

    return (unsigned char)((lb << 2) | rb);
}


int recall_forbidden_dir(unsigned int sig, unsigned char sidesig)
{
    unsigned int i;

    for (i = 0; i < DEADEND_MEMORY_N; i++) {
        if (de_valid[i] &&
            (de_sig[i] == sig) &&
            (de_sidesig[i] == sidesig)) {
            return de_last_dir[i];
        }
    }
    return MODE_STOP;
}

void remember_dead_end(unsigned int sig, unsigned char sidesig, unsigned char chosen_dir)
{
    unsigned int i;

    for (i = 0; i < DEADEND_MEMORY_N; i++) {
        if (de_valid[i] &&
            (de_sig[i] == sig) &&
            (de_sidesig[i] == sidesig)) {
            de_last_dir[i] = chosen_dir;
            return;
        }
    }

    de_sig[de_head] = sig;
    de_sidesig[de_head] = sidesig;
    de_last_dir[de_head] = chosen_dir;
    de_valid[de_head] = 1;
    de_head = (unsigned char)((de_head + 1) % DEADEND_MEMORY_N);
}

void forward_RC(unsigned int speed)
{
    base_speed = speed;
    vehicle_mode = MODE_FORWARD;
    current_direction = DIR_FORWARD;
    update_vehicle();
}

void backward_RC(unsigned int speed)
{
    if (!rear_is_safe()) {
        stop_RC();
        return;
    }
    base_speed = speed;
    vehicle_mode = MODE_BACKWARD;
    current_direction = DIR_BACKWARD;
    update_vehicle();
}


void soft_left_RC(unsigned int speed)
{
    base_speed = speed;
    vehicle_mode = MODE_FORWARD;
    current_direction = DIR_FORWARD;
    set_motor_dir(DIR_FORWARD);
    {
        unsigned int s = (speed > SOFT_TURN_DELTA) ? (speed - SOFT_TURN_DELTA) : 0;
        set_motor_speed(s, speed);
    }
}

void soft_right_RC(unsigned int speed)
{
    base_speed = speed;
    vehicle_mode = MODE_FORWARD;
    current_direction = DIR_FORWARD;
    set_motor_dir(DIR_FORWARD);
    {
        unsigned int s = (speed > SOFT_TURN_DELTA) ? (speed - SOFT_TURN_DELTA) : 0;
        set_motor_speed(speed, s);
    }
}


void wall_follow_right_p(unsigned int spd, unsigned int r, unsigned char have_r)
{
    int error;
    int corr;
    unsigned int left_pwm;
    unsigned int right_pwm;

    if ((have_r == 0) || (r < WALL_VALID_MIN_CM) || (r > WALL_VALID_MAX_CM)) {
        forward_RC(spd);
        return;
    }

    error = (int)r - (int)WALL_TARGET_CM;
    corr = error * WALL_KP;

    if (corr > WALL_CORR_MAX)  corr = WALL_CORR_MAX;
    if (corr < -WALL_CORR_MAX) corr = -WALL_CORR_MAX;

    if (corr >= 0) {


        left_pwm = spd;
        right_pwm = (spd > (unsigned int)corr + SPEED_MIN)
                        ? (spd - (unsigned int)corr) : SPEED_MIN;
    }
    else {

        corr = -corr;
        left_pwm = (spd > (unsigned int)corr + SPEED_MIN)
                        ? (spd - (unsigned int)corr) : SPEED_MIN;
        right_pwm = spd;
    }

    base_speed = spd;
    vehicle_mode = MODE_FORWARD;
    current_direction = DIR_FORWARD;
    set_motor_dir(DIR_FORWARD);
    set_motor_speed(left_pwm, right_pwm);
}


void wall_follow_center_or_right_p(unsigned int spd, unsigned int r, unsigned int l,
                                   unsigned char have_r, unsigned char have_l)
{
    int diff;
    int corr;
    unsigned int left_pwm;
    unsigned int right_pwm;

    if ((have_r == 1) && (have_l == 1) &&
        (r <= SIDE_CENTER_VALID_MAX_CM) && (l <= SIDE_CENTER_VALID_MAX_CM) &&
        ((r <= SIDE_CENTER_ACTIVE_CM) || (l <= SIDE_CENTER_ACTIVE_CM))) {

        diff = (int)l - (int)r;

        if ((diff >= SIDE_CENTER_DEADBAND_CM) ||
            (diff <= -SIDE_CENTER_DEADBAND_CM)) {

            if (diff > 0) {


                corr = diff * SIDE_CENTER_KP;
                if (corr > SIDE_CENTER_CORR_MAX) corr = SIDE_CENTER_CORR_MAX;

                left_pwm = (spd > (unsigned int)corr + SPEED_MIN)
                               ? (spd - (unsigned int)corr) : SPEED_MIN;
                right_pwm = spd;
            }
            else {


                corr = (-diff) * SIDE_CENTER_KP;
                if (corr > SIDE_CENTER_CORR_MAX) corr = SIDE_CENTER_CORR_MAX;

                left_pwm = spd;
                right_pwm = (spd > (unsigned int)corr + SPEED_MIN)
                                ? (spd - (unsigned int)corr) : SPEED_MIN;
            }

            base_speed = spd;
            vehicle_mode = MODE_FORWARD;
            current_direction = DIR_FORWARD;
            set_motor_dir(DIR_FORWARD);
            set_motor_speed(left_pwm, right_pwm);
            return;
        }
    }

    wall_follow_right_p(spd, r, have_r);
}

void left_RC(unsigned int speed)
{
    base_speed = speed;
    vehicle_mode = MODE_LEFT;
    current_direction = DIR_FORWARD;
    update_vehicle();
}

void right_RC(unsigned int speed)
{
    base_speed = speed;
    vehicle_mode = MODE_RIGHT;
    current_direction = DIR_FORWARD;
    update_vehicle();
}

void stop_RC(void)
{
    base_speed = SPEED_STOP;
    vehicle_mode = MODE_STOP;
    current_direction = DIR_STOP;
    update_vehicle();
}

void update_vehicle(void)
{
    unsigned int left_speed;
    unsigned int right_speed;

    if (vehicle_mode == MODE_STOP) {
        set_motor_speed(0, 0);
        set_motor_dir(DIR_STOP);
        return;
    }

    if (vehicle_mode == MODE_LEFT) {


        set_left_dir(DIR_BACKWARD);
        set_right_dir(DIR_FORWARD);
        set_motor_speed(TURN_BACKWARD_SPEED, TURN_FORWARD_SPEED);
        return;
    }
    else if (vehicle_mode == MODE_RIGHT) {

        set_left_dir(DIR_FORWARD);
        set_right_dir(DIR_BACKWARD);
        set_motor_speed(TURN_FORWARD_SPEED, TURN_BACKWARD_SPEED);
        return;
    }

    set_motor_dir(current_direction);
    left_speed = base_speed;
    right_speed = base_speed;
    set_motor_speed(left_speed, right_speed);
}

void set_left_dir(unsigned char direction)
{
    if (direction == DIR_FORWARD) {
        PORT->Group[0].OUTSET.reg = LEFT_IN1_MASK;
        PORT->Group[0].OUTCLR.reg = LEFT_IN2_MASK;
    }
    else if (direction == DIR_BACKWARD) {
        PORT->Group[0].OUTCLR.reg = LEFT_IN1_MASK;
        PORT->Group[0].OUTSET.reg = LEFT_IN2_MASK;
    }
    else {
        PORT->Group[0].OUTCLR.reg = LEFT_IN1_MASK;
        PORT->Group[0].OUTCLR.reg = LEFT_IN2_MASK;
    }
}

void set_right_dir(unsigned char direction)
{
    if (direction == DIR_FORWARD) {
        PORT->Group[0].OUTSET.reg = RIGHT_IN1_MASK;
        PORT->Group[0].OUTCLR.reg = RIGHT_IN2_MASK;
    }
    else if (direction == DIR_BACKWARD) {
        PORT->Group[0].OUTCLR.reg = RIGHT_IN1_MASK;
        PORT->Group[0].OUTSET.reg = RIGHT_IN2_MASK;
    }
    else {
        PORT->Group[0].OUTCLR.reg = RIGHT_IN1_MASK;
        PORT->Group[0].OUTCLR.reg = RIGHT_IN2_MASK;
    }
}

void set_motor_dir(unsigned char direction)
{
    set_left_dir(direction);
    set_right_dir(direction);
}

void set_motor_speed(unsigned int left_speed, unsigned int right_speed)
{
    if (left_speed > SPEED_MAX) left_speed = SPEED_MAX;
    if (right_speed > SPEED_MAX) right_speed = SPEED_MAX;

    TCC0->CC[2].reg = left_speed;
    while (TCC0->SYNCBUSY.bit.CC2) ;

    TCC0->CC[3].reg = right_speed;
    while (TCC0->SYNCBUSY.bit.CC3) ;
}


void process_user_command(void)
{


    volatile unsigned char rx_data;

    while (SERCOM5->USART.INTFLAG.bit.RXC == 1) {
        rx_data = (unsigned char)(SERCOM5->USART.DATA.reg & 0xFF);
        (void)rx_data;
    }
}


void cmd_speedup(void)
{
    if (base_speed + SPEED_STEP <= SPEED_MAX) {
        base_speed = base_speed + SPEED_STEP;
    }
    else {
        base_speed = SPEED_MAX;
    }
    update_vehicle();
}

void cmd_speeddown(void)
{
    if (base_speed > SPEED_MIN + SPEED_STEP) {
        base_speed = base_speed - SPEED_STEP;
    }
    else {
        base_speed = SPEED_MIN;
    }
    update_vehicle();
}


void console_write_char(unsigned char c)
{
    while (SERCOM5->USART.INTFLAG.bit.DRE != 1) ;
    SERCOM5->USART.DATA.reg = c;
}

void console_print_cstr(const char *s)
{
    unsigned int i = 0;
    while (s[i] != '\0') {
        console_write_char((unsigned char)s[i]);
        i++;
    }
}

void console_println_cstr(const char *s)
{
    console_print_cstr(s);
    console_write_char('\r');
    console_write_char('\n');
}

void console_print_unsigned_int(unsigned int data)
{
    char buf[11];
    int i;

    if (data == 0) {
        console_write_char('0');
        return;
    }

    i = 0;
    while ((data > 0) && (i < 10)) {
        buf[i] = (char)('0' + (data % 10));
        data = data / 10;
        i++;
    }

    while (i > 0) {
        i--;
        console_write_char((unsigned char)buf[i]);
    }
}

void console_print_int(int data)
{
    if (data < 0) {
        console_write_char('-');
        data = -data;
    }
    console_print_unsigned_int((unsigned int)data);
}

void console_print_status(void)
{
    console_print_cstr("Front=");
    if (front_cm == UNKNOWN_CM) console_print_cstr("unknown");
    else { console_print_unsigned_int(front_cm); console_print_cstr("cm"); }

    console_print_cstr(", RearUS=");
    if (rear_ultrasonic_cm == UNKNOWN_CM) console_print_cstr("unknown");
    else { console_print_unsigned_int(rear_ultrasonic_cm); console_print_cstr("cm"); }

    console_print_cstr(", Right=");
    if (right_cm == UNKNOWN_CM) console_print_cstr("unknown");
    else { console_print_unsigned_int(right_cm); console_print_cstr("cm"); }

    console_print_cstr(", Left=");
    if (left_cm == UNKNOWN_CM) console_print_cstr("unknown");
    else { console_print_unsigned_int(left_cm); console_print_cstr("cm"); }

    console_print_cstr(", Mode=");
    console_print_mode();

    console_print_cstr(", Speed=");
    console_print_int(speed_cms);
    console_print_cstr("cm/s");

    console_print_cstr(", Turns=");
    console_print_unsigned_int(turn_count);

    console_print_cstr(", Auto=");
    console_print_unsigned_int(auto_drive_enabled);

    console_print_cstr(", LidarBytes=");
    console_print_unsigned_int(lidar_rx_byte_count);

    console_print_cstr(", Headers=");
    console_print_unsigned_int(lidar_header_count);

    console_print_cstr(", VerLen=");
    console_print_unsigned_int(lidar_verlen_count);

    console_print_cstr(", Packets=");
    console_print_unsigned_int(lidar_packet_count);

    console_print_cstr(", Ang=");
    console_print_unsigned_int(lidar_last_start_angle);
    console_write_char('-');
    console_print_unsigned_int(lidar_last_end_angle);

    console_print_cstr(", Hits=");
    console_print_unsigned_int(lidar_front_hit_count);
    console_write_char('/');
    console_print_unsigned_int(lidar_right_hit_count);
    console_write_char('/');
    console_print_unsigned_int(lidar_left_hit_count);

    console_write_char('\r');
    console_write_char('\n');
}

void console_print_mode(void)
{
    if (vehicle_mode == MODE_STOP) console_print_cstr("STOP");
    else if (vehicle_mode == MODE_FORWARD) console_print_cstr("FORWARD");
    else if (vehicle_mode == MODE_BACKWARD) console_print_cstr("BACKWARD");
    else if (vehicle_mode == MODE_LEFT) console_print_cstr("TURN_LEFT");
    else if (vehicle_mode == MODE_RIGHT) console_print_cstr("TURN_RIGHT");
    else console_print_cstr("UNKNOWN");
}

void console_print_menu(void)
{
    console_println_cstr("---------------- Autonomous Demo ----------------");
    console_println_cstr("AUTO starts immediately after reset.");
    console_println_cstr("Manual serial driving commands are disabled.");
    console_println_cstr("Use reset/power switch to restart or stop the demo.");
    console_println_cstr("-------------------------------------------------");
}


void GCLK_setup(void)
{
    PM->APBAMASK.bit.GCLK_ = 1;
    PM->APBAMASK.bit.RTC_ = 1;
    PM->APBAMASK.bit.EIC_ = 1;

    PM->APBCMASK.bit.SERCOM5_ = 1;
    PM->APBCMASK.bit.SERCOM2_ = 1;
    PM->APBCMASK.bit.SERCOM3_ = 1;
    PM->APBCMASK.bit.TCC0_ = 1;

    SYSCTRL->OSC8M.bit.PRESC = 0;
    SYSCTRL->OSC8M.bit.ONDEMAND = 0;


    GCLK->GENCTRL.bit.ID = 2;
    GCLK->GENCTRL.bit.SRC = 6;
    GCLK->GENCTRL.bit.OE = 1;
    GCLK->GENCTRL.bit.GENEN = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) ;


    GCLK->CLKCTRL.bit.ID = 0x19;
    GCLK->CLKCTRL.bit.GEN = 2;
    GCLK->CLKCTRL.bit.CLKEN = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) ;


    GCLK->CLKCTRL.bit.ID = 0x16;
    GCLK->CLKCTRL.bit.GEN = 2;
    GCLK->CLKCTRL.bit.CLKEN = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) ;


    GCLK->CLKCTRL.bit.ID = 0x17;
    GCLK->CLKCTRL.bit.GEN = 2;
    GCLK->CLKCTRL.bit.CLKEN = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) ;


    GCLK->CLKCTRL.bit.ID = 0x13;
    GCLK->CLKCTRL.bit.GEN = 2;
    GCLK->CLKCTRL.bit.CLKEN = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) ;


    GCLK->CLKCTRL.bit.ID = 4;
    GCLK->CLKCTRL.bit.GEN = 2;
    GCLK->CLKCTRL.bit.CLKEN = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) ;


    GCLK->CLKCTRL.bit.ID = 5;
    GCLK->CLKCTRL.bit.GEN = 2;
    GCLK->CLKCTRL.bit.CLKEN = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) ;


    GCLK->CLKCTRL.bit.ID = 0x1A;
    GCLK->CLKCTRL.bit.GEN = 2;
    GCLK->CLKCTRL.bit.CLKEN = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) ;
}

void initializeSystemFor48MHz(void)
{
    NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val;

    PM->APBAMASK.bit.GCLK_ = 1;

    GCLK->CTRL.bit.SWRST = 1;
    while (GCLK->CTRL.bit.SWRST && GCLK->STATUS.bit.SYNCBUSY) ;

    SYSCTRL->OSC8M.bit.ONDEMAND = 0;

    GCLK->GENDIV.reg = GCLK_GENDIV_ID(3) | GCLK_GENDIV_DIV(64);
    while (GCLK->STATUS.bit.SYNCBUSY) ;

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(3) | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.bit.SYNCBUSY) ;

    GCLK->CLKCTRL.reg = 0x00 | GCLK_CLKCTRL_GEN(3) | GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY) ;

    SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY) ;

    SYSCTRL->DFLLMUL.bit.MUL = 3072;
    SYSCTRL->DFLLMUL.bit.CSTEP = 1;
    SYSCTRL->DFLLMUL.bit.FSTEP = 1;
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY) ;

    SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_QLDIS;
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY) ;

    SYSCTRL->INTFLAG.reg =
        SYSCTRL_INTFLAG_DFLLLCKC | SYSCTRL_INTFLAG_DFLLLCKF | SYSCTRL_INTFLAG_DFLLRDY;

    SYSCTRL->DFLLCTRL.bit.ENABLE = 1;
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY) ;

    while (!SYSCTRL->INTFLAG.bit.DFLLLCKC && !SYSCTRL->INTFLAG.bit.DFLLLCKF) ;
    while (!SYSCTRL->INTFLAG.bit.DFLLRDY) ;

    GCLK->GENDIV.reg = GCLK_GENDIV_ID(0) | GCLK_GENDIV_DIV(0);
    while (GCLK->STATUS.bit.SYNCBUSY) ;

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M |
                        GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
    GCLK->GENCTRL.bit.OE = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) ;
}


void PORT_setup(void)
{

    PORT->Group[0].PINCFG[14].reg = 0x41;
    PORT->Group[0].PMUX[7].bit.PMUXE = 0x07;


    PORT->Group[0].PINCFG[20].reg = 0x41;
    PORT->Group[0].PINCFG[21].reg = 0x41;
    PORT->Group[0].PMUX[10].bit.PMUXE = 0x03;
    PORT->Group[0].PMUX[10].bit.PMUXO = 0x03;


    PORT->Group[0].PINCFG[10].reg = 0x41;
    PORT->Group[0].PINCFG[11].reg = 0x41;
    PORT->Group[0].PMUX[5].bit.PMUXE = 0x03;
    PORT->Group[0].PMUX[5].bit.PMUXO = 0x03;


    PORT->Group[0].PINCFG[22].reg = 0x41;
    PORT->Group[0].PINCFG[23].reg = 0x41;
    PORT->Group[0].PMUX[11].bit.PMUXE = 0x02;
    PORT->Group[0].PMUX[11].bit.PMUXO = 0x02;


    PORT->Group[0].PINCFG[TRIG_PIN].reg = 0x00;
    PORT->Group[0].DIRSET.reg = TRIG_MASK;
    PORT->Group[0].OUTCLR.reg = TRIG_MASK;


    PORT->Group[0].PINCFG[ECHO_PIN].reg = 0x41;
    PORT->Group[0].PMUX[8].bit.PMUXE = 0x00;


    PORT->Group[0].PINCFG[LEFT_IN1_PIN].reg = 0x00;
    PORT->Group[0].PINCFG[LEFT_IN2_PIN].reg = 0x00;
    PORT->Group[0].PINCFG[RIGHT_IN1_PIN].reg = 0x00;
    PORT->Group[0].PINCFG[RIGHT_IN2_PIN].reg = 0x00;
    PORT->Group[0].DIRSET.reg = MOTOR_DIR_MASK;
    PORT->Group[0].OUTCLR.reg = MOTOR_DIR_MASK;
}

void I2C_setup(void)
{
    SERCOM3->I2CM.CTRLA.bit.SWRST = 1;
    while (SERCOM3->I2CM.CTRLA.bit.SWRST) ;

    SERCOM3->I2CM.CTRLA.bit.ENABLE = 0;
    SERCOM3->I2CM.CTRLA.bit.MODE = 0x5;
    SERCOM3->I2CM.BAUD.bit.BAUD = 0x20;

    SERCOM3->I2CM.INTENCLR.bit.MB = 1;
    SERCOM3->I2CM.INTENCLR.bit.SB = 1;
    SERCOM3->I2CM.INTENCLR.bit.ERROR = 1;

    SERCOM3->I2CM.CTRLA.bit.ENABLE = 1;
    while (SERCOM3->I2CM.SYNCBUSY.bit.ENABLE) ;

    SERCOM3->I2CM.STATUS.bit.BUSSTATE = 1;
    while (SERCOM3->I2CM.SYNCBUSY.bit.SYSOP) ;
}

void USART_setup(void)
{

    SERCOM5->USART.CTRLA.bit.MODE = 1;
    SERCOM5->USART.CTRLA.bit.CMODE = 0;
    SERCOM5->USART.CTRLA.bit.RXPO = 3;
    SERCOM5->USART.CTRLA.bit.TXPO = 1;
    SERCOM5->USART.CTRLB.bit.CHSIZE = 0;
    SERCOM5->USART.CTRLA.bit.DORD = 1;
    SERCOM5->USART.BAUD.reg = CONSOLE_BAUD;
    SERCOM5->USART.CTRLB.bit.RXEN = 1;
    SERCOM5->USART.CTRLB.bit.TXEN = 1;
    SERCOM5->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE) ;


    SERCOM2->USART.CTRLA.bit.MODE = 1;
    SERCOM2->USART.CTRLA.bit.CMODE = 0;
    SERCOM2->USART.CTRLA.bit.RXPO = 3;
    SERCOM2->USART.CTRLA.bit.TXPO = 1;
    SERCOM2->USART.CTRLB.bit.CHSIZE = 0;
    SERCOM2->USART.CTRLA.bit.DORD = 1;
    SERCOM2->USART.BAUD.reg = BAUD_230400;
    SERCOM2->USART.CTRLB.bit.RXEN = 1;
    SERCOM2->USART.CTRLB.bit.TXEN = 1;
    SERCOM2->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM2->USART.SYNCBUSY.bit.ENABLE) ;
}

void EIC_setup(void)
{


    EIC->CONFIG[0].bit.FILTEN0 = 1;
    EIC->CONFIG[0].bit.SENSE0 = 0x3;
    EIC->INTENSET.bit.EXTINT0 = 1;
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.bit.SYNCBUSY) ;
}


void RTC_setup_10us(void)
{
    RTC->MODE0.CTRL.bit.ENABLE = 0;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY) ;

    RTC->MODE0.CTRL.bit.SWRST = 1;
    while (RTC->MODE0.CTRL.bit.SWRST) ;

    RTC->MODE0.CTRL.bit.MODE = 0;
    RTC->MODE0.CTRL.bit.PRESCALER = 0;
    RTC->MODE0.CTRL.bit.MATCHCLR = 0;
    RTC->MODE0.COUNT.reg = 0;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY) ;

    RTC->MODE0.CTRL.bit.ENABLE = 1;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY) ;
}

void TCC0_setup(void)
{


    PORT->Group[0].PINCFG[LEFT_PWM_PIN].reg = 0x41;
    PORT->Group[0].PMUX[9].bit.PMUXE = 0x05;

    PORT->Group[0].PINCFG[RIGHT_PWM_PIN].reg = 0x41;
    PORT->Group[0].PMUX[9].bit.PMUXO = 0x05;

    TCC0->CTRLA.bit.ENABLE = 0;
    while (TCC0->SYNCBUSY.bit.ENABLE) ;

    TCC0->CTRLA.bit.PRESCALER = 6;
    TCC0->WAVE.bit.WAVEGEN = 2;
    while (TCC0->SYNCBUSY.bit.WAVE) ;

    TCC0->PER.reg = PWM_PERIOD;
    while (TCC0->SYNCBUSY.bit.PER) ;

    TCC0->CC[2].reg = 0;
    while (TCC0->SYNCBUSY.bit.CC2) ;
    TCC0->CC[3].reg = 0;
    while (TCC0->SYNCBUSY.bit.CC3) ;

    TCC0->CTRLA.bit.ENABLE = 1;
    while (TCC0->SYNCBUSY.bit.ENABLE) ;
}
