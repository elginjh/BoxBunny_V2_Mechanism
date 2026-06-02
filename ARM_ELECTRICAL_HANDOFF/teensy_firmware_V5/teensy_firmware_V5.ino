#include <micro_ros_arduino.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <Wire.h>

// --- COMPATIBILITY PATCH ---
#include <ctype.h>
#undef __locale_ctype_ptr
extern "C" {
  const unsigned char * __locale_ctype_ptr (void) { return (const unsigned char *)_ctype_; }
}
// ---------------------------

#define NONE RCLC_NONE_PLACEHOLDER
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#undef NONE

#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/string.h>
#include <FlexCAN_T4.h>

// =============================================================================
// FIRMWARE V4 — Motor Control + IMU Integration (200Hz Unified Loop)
// =============================================================================
// Extends V3 with:
//   - I2C polling of up to 4 MPU6050 IMUs (Wire + Wire1, 0x68 + 0x69)
//   - Extended feedback payload: 21 doubles (motor pos/curr + IMU accel)
//   - 200Hz unified control + feedback loop (5ms intervals)
// =============================================================================


// --- HARDWARE CONFIG ---
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

const int ID_LEFT_1  = 0x01;
const int ID_LEFT_2  = 0x02;
const int ID_RIGHT_1 = 0x03;
const int ID_RIGHT_2 = 0x04;

// --- IMU CONFIG ---
const int MPU_ADDR_1 = 0x68;
const int MPU_ADDR_2 = 0x69;
bool imu1_active = false;   // Wire,  0x68
bool imu2_active = false;   // Wire,  0x69
bool imu3_active = false;   // Wire1, 0x68
bool imu4_active = false;   // Wire1, 0x69

// IMU acceleration data (m/s²), updated each loop
float imu_accel[4][3] = {{0}};   // [imu_index][axis: x,y,z]

// --- DYNAMIC MECHANICAL LIMITS (Kinematic Decoupling) ---
void apply_dynamic_endstops(float targets[4]) {
    const float GLOBAL_SAFE_MAX = 50.0;
    const float GLOBAL_SAFE_MIN = -50.0;
    for (int i = 0; i < 4; i++) {
        if (targets[i] > GLOBAL_SAFE_MAX) targets[i] = GLOBAL_SAFE_MAX;
        if (targets[i] < GLOBAL_SAFE_MIN) targets[i] = GLOBAL_SAFE_MIN;
    }
}

// --- DATA GLOBALS ---
float target_pos[4] = {0.0};
float target_speed[4] = {5.0, 5.0, 5.0, 5.0};
float actual_pos[4] = {0.0};
float actual_current[4] = {0.0};
float last_sent_pos[4] = {-9999.0, -9999.0, -9999.0, -9999.0};

bool motors_enabled = false;
uint32_t can_rx_count = 0;


// --- IMU FUNCTIONS ---
bool initMPU(TwoWire &wire, int addr) {
    wire.beginTransmission(addr);
    if (wire.endTransmission() != 0) return false;

    wire.beginTransmission(addr);
    wire.write(0x6B);  // PWR_MGMT_1
    wire.write(0);     // Wake up
    wire.endTransmission(true);

    wire.beginTransmission(addr);
    wire.write(0x1B);  // GYRO_CONFIG
    wire.write(0x00);  // ±250 dps
    wire.endTransmission(true);

    wire.beginTransmission(addr);
    wire.write(0x1C);  // ACCEL_CONFIG
    wire.write(0x18);  // ±16g (FS_SEL = 11) — full range for boxing impact detection
    wire.endTransmission(true);

    return true;
}

// Track consecutive errors per IMU slot to trigger auto-recovery
uint16_t imu_err_count[4] = {0, 0, 0, 0};
bool imu_healthy = true;

void readMPU(TwoWire &wire, int addr, int16_t* data, int slot_idx) {
    // If an IMU has been offline/failing heavily, attempt to re-awaken it every 50 loops (250ms)
    if (imu_err_count[slot_idx] > 10) {
        if (imu_err_count[slot_idx] % 50 == 0) {
            initMPU(wire, addr); // Send wake-up and config registers again
        }
    }

    wire.beginTransmission(addr);
    wire.write(0x3B);
    
    // Force a true STOP condition.
    if (wire.endTransmission(true) != 0) {
        for(int i=0; i<6; i++) data[i] = 0;
        imu_err_count[slot_idx]++;
        return;
    }
    
    // Validate that all 14 bytes successfully arrived.
    if (wire.requestFrom((uint8_t)addr, (size_t)14, true) == 14) {
        data[0] = wire.read() << 8 | wire.read();  // Accel X
        data[1] = wire.read() << 8 | wire.read();  // Accel Y
        data[2] = wire.read() << 8 | wire.read();  // Accel Z
        wire.read() << 8 | wire.read();             // Temperature (discard)
        data[3] = wire.read() << 8 | wire.read();  // Gyro X
        data[4] = wire.read() << 8 | wire.read();  // Gyro Y
        data[5] = wire.read() << 8 | wire.read();  // Gyro Z
        imu_err_count[slot_idx] = 0; // Success, reset error counter
    } else {
        for(int i=0; i<6; i++) data[i] = 0;
        while(wire.available()) wire.read(); // Drain bus
        imu_err_count[slot_idx]++;
    }
}

void readAllIMUs() {
    int16_t d1[6], d2[6], d3[6], d4[6];
    
    // Poll all 4 slots regardless of initial boot state (supports hot-plugging)
    readMPU(Wire,  MPU_ADDR_1, d1, 0);
    readMPU(Wire,  MPU_ADDR_2, d2, 1);
    readMPU(Wire1, MPU_ADDR_1, d3, 2);
    readMPU(Wire1, MPU_ADDR_2, d4, 3);

    // Convert raw int16 to m/s² (±16g range: 2048 LSB/g)
    int16_t* all[4] = {d1, d2, d3, d4};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            imu_accel[i][j] = ((float)all[i][j] / 2048.0f) * 9.81f;
        }
    }

    // Evaluate Global IMU Health
    imu_healthy = true;
    for (int i = 0; i < 4; i++) {
        // If an IMU fails to respond more than 3 consecutive times, mark system as unhealthy
        if (imu_err_count[i] > 3) {
            imu_healthy = false;
        }
    }
}


// --- ROS GLOBALS ---
rcl_subscription_t subscriber;
std_msgs__msg__Float64MultiArray msg_sub;
rcl_publisher_t feedback_pub;
std_msgs__msg__Float64MultiArray msg_feedback;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

static double msg_data_buffer[9];
static double feedback_data_buffer[21];   // Extended: 9 motor + 12 IMU

#define LED_PIN 13

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return false; }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void set_motor_state(uint16_t id, bool enable) {
    CAN_message_t msg;
    msg.id = id;
    msg.len = 8;
    for(int i=0; i<7; i++) msg.buf[i] = 0xFF;
    msg.buf[7] = enable ? 0xFC : 0xFD;
    Can0.write(msg);
}

void pack_and_send_pos_speed(uint8_t id, float p_des, float v_des) {
    CAN_message_t msg;
    msg.id = 0x100 + id;
    msg.len = 8;
    memcpy(&msg.buf[0], &p_des, 4);
    memcpy(&msg.buf[4], &v_des, 4);
    Can0.write(msg);
}

void read_can_feedback() {
    CAN_message_t msg;
    while(Can0.read(msg)) {
        can_rx_count++;

        int id_payload = msg.buf[0] & 0x0F;

        uint16_t p_int = ((uint16_t)msg.buf[1] << 8) | (uint16_t)msg.buf[2];
        float p = uint_to_float(p_int, -12.5663706f, 12.5663706f, 16);

        uint16_t t_int = (((uint16_t)(msg.buf[4] & 0x0F)) << 8) | (uint16_t)msg.buf[5];
        float t = uint_to_float(t_int, -10.0f, 10.0f, 12);

        int idx = -1;
        if (msg.id == ID_LEFT_1 || id_payload == ID_LEFT_1) idx = 0;
        else if (msg.id == ID_LEFT_2 || id_payload == ID_LEFT_2) idx = 1;
        else if (msg.id == ID_RIGHT_1 || id_payload == ID_RIGHT_1) idx = 2;
        else if (msg.id == ID_RIGHT_2 || id_payload == ID_RIGHT_2) idx = 3;

        if (idx != -1) {
            actual_pos[idx] = p;
            actual_current[idx] = t;
        }
    }
}

void subscription_callback(const void * msin) {
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msin;

  if (msg->data.size >= 4) {
    for(int i=0; i<4; i++) {
        target_pos[i] = (float)msg->data.data[i];
    }
    apply_dynamic_endstops(target_pos);
  }
  if (msg->data.size >= 8) {
    for(int i=0; i<4; i++) target_speed[i] = (float)msg->data.data[4+i];
  }
  if (msg->data.size >= 9) {
    bool enable = (msg->data.data[8] > 0.5);
    if (enable != motors_enabled) {
        motors_enabled = enable;
        if(motors_enabled) {
             for(int i=0; i<4; i++) target_pos[i] = actual_pos[i];
             for(int i=0; i<5; i++) {
                set_motor_state(ID_LEFT_1, true); set_motor_state(ID_LEFT_2, true);
                set_motor_state(ID_RIGHT_1, true); set_motor_state(ID_RIGHT_2, true);
                delay(2);
             }
        } else {
             set_motor_state(ID_LEFT_1, false); set_motor_state(ID_LEFT_2, false);
             set_motor_state(ID_RIGHT_1, false); set_motor_state(ID_RIGHT_2, false);
        }
    }
  }
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "dual_arm_bridge", "", &support));

  std_msgs__msg__Float64MultiArray__init(&msg_sub);
  msg_sub.data.capacity = 9; msg_sub.data.data = msg_data_buffer; msg_sub.data.size = 0;

  RCCHECK(rclc_subscription_init_default(&subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "motor_commands"));

  std_msgs__msg__Float64MultiArray__init(&msg_feedback);
  msg_feedback.data.capacity = 21;
  msg_feedback.data.data = feedback_data_buffer;
  msg_feedback.data.size = 0;

  RCCHECK(rclc_publisher_init_default(&feedback_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "motor_feedback"));



  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities() {
  RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));

  RCSOFTCHECK(rcl_publisher_fini(&feedback_pub, &node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void setup() {
  pinMode(LED_PIN, OUTPUT);

  // CAN Bus
  Can0.begin(); Can0.setBaudRate(1000000); Can0.setMBFilter(ACCEPT_ALL);

  // I2C Buses for IMUs (400kHz Fast Mode)
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true); // 3ms timeout, reset on timeout
  Wire1.begin();
  Wire1.setClock(400000);
  Wire1.setWireTimeout(3000, true); // 3ms timeout, reset on timeout

  // Probe and initialize all IMU slots
  imu1_active = initMPU(Wire,  MPU_ADDR_1);
  imu2_active = initMPU(Wire,  MPU_ADDR_2);
  imu3_active = initMPU(Wire1, MPU_ADDR_1);
  imu4_active = initMPU(Wire1, MPU_ADDR_2);

  // micro-ROS transport
  Serial.begin(115200);
  set_microros_transports();
  state = WAITING_AGENT;
}

unsigned long last_ping = 0;
unsigned long last_feedback = 0;

void loop() {
  read_can_feedback();

  switch (state) {
    case WAITING_AGENT:
      if (millis() - last_ping > 100) {
          last_ping = millis();
          if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }
      break;
    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == AGENT_CONNECTED) digitalWrite(LED_PIN, HIGH);
      break;
    case AGENT_CONNECTED:
      // Spin quickly, do not block the 200Hz loop
      if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)) != RCL_RET_OK) {
         SCB_AIRCR = 0x05FA0004;  // Hard Robot CPU Reboot
      }

      // Periodically verify agent is still alive so we don't hang if it's restarted via Ctrl+C
      static unsigned long last_agent_check = 0;
      static int missed_pings = 0;
      
      if (millis() - last_agent_check >= 500) {
          last_agent_check = millis();
          // Relaxed ping (10ms timeout)
          if (rmw_uros_ping_agent(10, 1) != RMW_RET_OK) {
              missed_pings++;
              if (missed_pings >= 3) {
                  // Hard reboot the Teensy 4.0 (ARM Cortex-M7) to cleanly obliterate the micro-ROS session
                  SCB_AIRCR = 0x05FA0004; 
              }
          } else {
              missed_pings = 0;
          }
      }

      // === Unified 200Hz Control + Feedback Loop (5ms interval) ===
      static unsigned long last_ctrl = 0;
      if (millis() - last_ctrl >= 5) {
         last_ctrl = millis();

         // --- FIRMWARE CURRENT WATCHDOG (200Hz) ---
         // Last line of defense: if any motor exceeds the hard limit,
         // immediately disable all motors regardless of GUI state.
         const float CURRENT_LIMIT_A = 3.0;  // Amps — matches GUI config
         if (motors_enabled) {
             for (int i = 0; i < 4; i++) {
                 if (fabsf(actual_current[i]) > CURRENT_LIMIT_A) {
                     motors_enabled = false;
                     set_motor_state(ID_LEFT_1, false);
                     set_motor_state(ID_LEFT_2, false);
                     set_motor_state(ID_RIGHT_1, false);
                     set_motor_state(ID_RIGHT_2, false);
                     for (int j = 0; j < 4; j++) {
                         target_pos[j] = actual_pos[j];
                         last_sent_pos[j] = -9999.0;
                     }
                     break;  // Motors disabled, exit check
                 }
             }
         }

         // --- SOFTWARE FAILSAFE: Sensor State Override ---
         if (!imu_healthy && motors_enabled) {
             motors_enabled = false;
         }

         // --- Motor Control ---
         if (motors_enabled) {
             pack_and_send_pos_speed(ID_LEFT_1, target_pos[0], target_speed[0]);
             pack_and_send_pos_speed(ID_LEFT_2, target_pos[1], target_speed[1]);
             pack_and_send_pos_speed(ID_RIGHT_1, target_pos[2], target_speed[2]);
             pack_and_send_pos_speed(ID_RIGHT_2, target_pos[3], target_speed[3]);
             for(int i=0; i<4; i++) last_sent_pos[i] = target_pos[i];
         } else {
             set_motor_state(ID_LEFT_1, false); set_motor_state(ID_LEFT_2, false);
             set_motor_state(ID_RIGHT_1, false); set_motor_state(ID_RIGHT_2, false);
             for(int i=0; i<4; i++) {
                 target_pos[i] = actual_pos[i];
                 last_sent_pos[i] = -9999.0;
             }
         }

         // --- IMU Read ---
         readAllIMUs();

         // --- Publish Extended Feedback ---
         // [0..3] motor positions, [4..7] motor currents, [8] CAN count,
         // [9..11] IMU1 accel XYZ, [12..14] IMU2, [15..17] IMU3, [18..20] IMU4
         msg_feedback.data.size = 21;
         for(int i=0; i<4; i++) msg_feedback.data.data[i]   = actual_pos[i];
         for(int i=0; i<4; i++) msg_feedback.data.data[4+i] = actual_current[i];
         msg_feedback.data.data[8] = (double)can_rx_count;
         for(int imu=0; imu<4; imu++) {
             for(int ax=0; ax<3; ax++) {
                 msg_feedback.data.data[9 + imu*3 + ax] = (double)imu_accel[imu][ax];
             }
         }
         RCSOFTCHECK(rcl_publish(&feedback_pub, &msg_feedback, NULL));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
