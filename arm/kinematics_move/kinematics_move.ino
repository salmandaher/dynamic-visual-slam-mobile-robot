#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/point.h>

#include "ESPMax.h"
#include "_espmax.h"
#include "Buzzer.h"

// micro-ROS handles
rcl_subscription_t subscriber;
geometry_msgs__msg__Point msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float pos[3];
bool new_msg = false;

bool inRange(float x, float y, float z) {
    if (x < -291.34 || x > 291.34) return false;
    if (y < -291.34 || y > 291.34) return false;
    if (z < -182.0  || z > 350.8)  return false;
    return true;
}

// Called every time a message arrives on /arm_position
void subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
    pos[0] = msg->x;
    pos[1] = msg->y;
    pos[2] = msg->z;
    new_msg = true;
}

void setup() {
    set_microros_transports();
    delay(2000);

    Buzzer_init();
    ESPMax_init();
    go_home(2000);
    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "espmax_node", "", &support);

    rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
        "/arm_position"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg,
        &subscription_callback, ON_NEW_DATA);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    if (new_msg) {
        new_msg = false;

        if (!inRange(pos[0], pos[1], pos[2])) {
            // out of range — ignore silently or beepError()
            return;
        }

        set_position(pos, 2000);
        delay(2000);
    }
}