// espmax_passthrough — ESP32 micro-ROS joint/servo pass-through for the ESPMax arm.
//
// Replaces the stock kinematics_move.ino "/arm_position -> onboard IK" behaviour.
// Here the ESP32 is a DUMB actuator bridge: the ROS host owns all kinematics and
// calibration; this firmware only moves servos to commanded pulses and reports
// the pulses it reads back. This is the "direct servo driver" path.
//
// Contract with the host:
//   sub  /arm/servo_command  std_msgs/Float32MultiArray  data=[p1, p2, p3, time_ms]
//        p1,p2,p3 = Lobot pulses (0..1000) for servos 1,2,3; time_ms = move duration.
//   pub  /arm/servo_feedback std_msgs/Float32MultiArray  data=[p1, p2, p3]
//        pulses read back from servos 1,2,3, published at ~FEEDBACK_HZ.
//
// Safety: pulses are clamped to [0,1000] AND to the company's mechanical guards
// (servo2 <= 700, servo3 >= 470) so we never drive the linkage into itself.
//
// Build: place this .ino with the company's LobotSerialServoControl.{h,cpp} in the
// same sketch folder (copy them from ../../kinematics_move/). Requires the
// micro_ros_arduino library. Original kinematics_move.ino is left untouched.

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "LobotSerialServoControl.h"

// ---- servo bus wiring (matches the company ESPMax.cpp) ----
#define SERVO_SERIAL_RX 35
#define SERVO_SERIAL_TX 12
// Half-duplex bus direction-enable pins. The company ESPMax.cpp #defines these
// (receiveEnablePin 13, transmitEnablePin 14) but constructs BusServo with the
// single-arg (auto-direction) constructor, so they are never driven -- which is
// why writes work (TX defaults enabled) but reads return nothing (RX never
// enabled). Passing them to the 3-arg constructor makes the library flip
// direction (TxEnable before write, RxEnable before read) so feedback works.
#define SERVO_RX_ENABLE 13
#define SERVO_TX_ENABLE 14
HardwareSerial ServoSerial(2);
LobotSerialServoControl BusServo(ServoSerial, SERVO_RX_ENABLE, SERVO_TX_ENABLE);

// ---- behaviour ----
#define NUM_SERVOS   3
#define FEEDBACK_HZ  20
#define DEFAULT_TIME 1000   // ms, used if the host omits a move time

// ---- micro-ROS handles ----
rcl_subscription_t sub_cmd;
rcl_publisher_t    pub_fb;
std_msgs__msg__Float32MultiArray msg_cmd;
std_msgs__msg__Float32MultiArray msg_fb;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     fb_timer;

// static backing storage for the Float32MultiArray payloads (micro-ROS needs the
// sequence buffers pre-allocated; no dynamic allocation at runtime).
static float cmd_buf[4];   // [p1, p2, p3, time_ms]
static float fb_buf[NUM_SERVOS];

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Clamp a pulse to [0,1000] and apply the company's per-servo mechanical guards.
static int safe_pulse(int servo_id, float pulse) {
    int p = (int)(pulse + 0.5f);
    p = (int)clampf((float)p, 0.0f, 1000.0f);
    if (servo_id == 2 && p > 700) p = 700;   // shoulder upper guard
    if (servo_id == 3 && p < 470) p = 470;   // elbow lower guard
    return p;
}

// Called when the host sends a new servo command.
void command_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * m =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    if (m->data.size < NUM_SERVOS) return;   // ignore malformed commands

    uint16_t move_time = DEFAULT_TIME;
    if (m->data.size >= 4 && m->data.data[3] > 0.0f) {
        move_time = (uint16_t)clampf(m->data.data[3], 20.0f, 30000.0f);
    }
    for (int i = 0; i < NUM_SERVOS; i++) {
        int p = safe_pulse(i + 1, m->data.data[i]);
        BusServo.LobotSerialServoMove(i + 1, p, move_time);
        delay(2);   // small gap between bus writes, as in the company code
    }
}

// Publish read-back pulses on a fixed timer.
void feedback_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;
    for (int i = 0; i < NUM_SERVOS; i++) {
        int p = BusServo.LobotSerialServoReadPosition(i + 1);
        fb_buf[i] = (float)p;   // negative => read error; host can detect/ignore
    }
    msg_fb.data.size = NUM_SERVOS;
    rcl_publish(&pub_fb, &msg_fb, NULL);
}

void setup() {
    set_microros_transports();
    delay(2000);

    BusServo.OnInit();
    ServoSerial.begin(115200, SERIAL_8N1, SERVO_SERIAL_RX, SERVO_SERIAL_TX);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "espmax_joint_node", "", &support);

    // bind the static buffers to the message sequences
    msg_cmd.data.data     = cmd_buf;
    msg_cmd.data.capacity = 4;
    msg_cmd.data.size     = 0;
    msg_fb.data.data      = fb_buf;
    msg_fb.data.capacity  = NUM_SERVOS;
    msg_fb.data.size      = NUM_SERVOS;

    rclc_subscription_init_default(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/arm/servo_command");

    rclc_publisher_init_default(
        &pub_fb, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/arm/servo_feedback");

    rclc_timer_init_default(
        &fb_timer, &support, RCL_MS_TO_NS(1000 / FEEDBACK_HZ),
        feedback_timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd,
        &command_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &fb_timer);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
}
