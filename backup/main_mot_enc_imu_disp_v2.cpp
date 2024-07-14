#include <Arduino.h>                            // needed if using Platformio and VS Code IDE
#include <micro_ros_platformio.h>               // use if using platformio, otherwise #include <miro_ros_arduino.h>
#include <Wire.h>
#include <esp32-hal-ledc.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>

// BNO055 IMU
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BusIO_Register.h>
//#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <utility/imumaths.h>

// OLED display settings
#include <Adafruit_SSD1306.h>
#include <std_msgs/msg/float32_multi_array.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C // Define the I2C address for your display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Motor Pins
#define MOTOR1_PWM  14
#define MOTOR2_PWM  27
#define MOTOR3_PWM  26
#define MOTOR4_PWM  25
#define MOTOR1_DIR  12
#define MOTOR2_DIR  13
#define MOTOR3_DIR  17
#define MOTOR4_DIR  18
#define MOTOR1_ENC_A 34
#define MOTOR1_ENC_B 35
#define MOTOR2_ENC_A 32
#define MOTOR2_ENC_B 33
#define MOTOR3_ENC_A 36
#define MOTOR3_ENC_B 39
#define MOTOR4_ENC_A 23
#define MOTOR4_ENC_B 19 // was 22 but clashes with SCL for IMU
#define LED 2

// IMU objects and variables
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
rcl_publisher_t imu_publisher;
//geometry_msgs__msg__Vector3 imu_msg;
sensor_msgs__msg__Imu imu_msg;


void setup_imu() {
    // Initialize I2C communication
    Wire.begin(21, 22);

    // Initialize the BNO055 sensor
    if (!bno.begin()) {
        Serial.println("No BNO055 detected. Check your wiring or I2C ADDR!");
        while (1);
    }

    // Set up the BNO055 sensor
    bno.setExtCrystalUse(true);

    imu_msg.orientation_covariance[0] = -1; // Indicates orientation covariance is not provided
    imu_msg.angular_velocity_covariance[0] = -1; // Indicates angular velocity covariance is not provided
    imu_msg.linear_acceleration_covariance[0] = -1; // Indicates linear acceleration covariance is not provided
}

void setup_oled_display() {
    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.display();
    delay(2000);
    display.clearDisplay();
    display.display();
}

// Wheel encoder variables
volatile int32_t wheel1_count = 0;
volatile int32_t wheel2_count = 0;
volatile int32_t wheel3_count = 0;
volatile int32_t wheel4_count = 0;

// Motor speed and direction
int motor1_speed = 0;
int motor2_speed = 0;
int motor3_speed = 0;
int motor4_speed = 0;

// Declare the encoder publisher oject and message array
rcl_publisher_t encoder_publisher;
std_msgs__msg__Int32MultiArray encoder_msg;

// Declare the cmd_vel subscriber and twist message variable
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

void setup_motors() {
    ledcAttachPin(MOTOR1_PWM, 0);
    ledcAttachPin(MOTOR2_PWM, 1);
    ledcAttachPin(MOTOR3_PWM, 2);
    ledcAttachPin(MOTOR4_PWM, 3);

    ledcSetup(0, 5000, 8); // channel 0, 5kHz PWM, 8-bit resolution
    ledcSetup(1, 5000, 8); // channel 1, 5kHz PWM, 8-bit resolution
    ledcSetup(2, 5000, 8); // channel 2, 5kHz PWM, 8-bit resolution
    ledcSetup(3, 5000, 8); // channel 3, 5kHz PWM, 8-bit resolution

    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(MOTOR4_DIR, OUTPUT);
}

void set_motor_speed(int motor, int speed) {
    int pwm_value = abs(speed);
    bool direction = (speed >= 0);

    switch (motor) {
        case 0:
            digitalWrite(MOTOR1_DIR, direction);
            ledcWrite(0, pwm_value);
            motor1_speed = speed;
            break;
        case 1:
            digitalWrite(MOTOR2_DIR, direction);
            ledcWrite(1, pwm_value);
            motor2_speed = speed;
            break;
        case 2:
            digitalWrite(MOTOR3_DIR, direction);
            ledcWrite(2, pwm_value);
            motor3_speed = speed;
            break;
        case 3:
            digitalWrite(MOTOR4_DIR, direction);
            ledcWrite(3, pwm_value);
            motor4_speed = speed;
            break;
    }
}

void encoder_isr_1() {
    if (motor1_speed >= 0) {
        wheel1_count++;
    } else {
        wheel1_count--;
    }
}

void encoder_isr_2() {
    if (motor2_speed >= 0) {
        wheel2_count++;
    } else {
        wheel2_count--;
    }
}

void encoder_isr_3() {
    if (motor3_speed >= 0) {
        wheel3_count++;
    } else {
        wheel3_count--;
    }
}

void encoder_isr_4() {
    if (motor4_speed >= 0) {
        wheel4_count++;
    } else {
        wheel4_count--;
    }
}

void setup_encoders() {
    pinMode(MOTOR1_ENC_A, INPUT);
    pinMode(MOTOR1_ENC_B, INPUT);
    pinMode(MOTOR2_ENC_A, INPUT);
    pinMode(MOTOR2_ENC_B, INPUT);
    pinMode(MOTOR3_ENC_A, INPUT);
    pinMode(MOTOR3_ENC_B, INPUT);
    pinMode(MOTOR4_ENC_A, INPUT);
    pinMode(MOTOR4_ENC_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_A), encoder_isr_1, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_A), encoder_isr_2, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR3_ENC_A), encoder_isr_3, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR4_ENC_A), encoder_isr_4, RISING);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    
    // Update encoder message
    encoder_msg.data.data[0] = wheel1_count;
    encoder_msg.data.data[1] = wheel2_count;
    encoder_msg.data.data[2] = wheel3_count;
    encoder_msg.data.data[3] = wheel4_count;

    // Publish encoder data
    rcl_ret_t ret_enc_ok = rcl_publish(&encoder_publisher, &encoder_msg, NULL);

    // ***************  Read the BNO055 sensor data  ************************************
    // Get the BNO055 calibration status
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    // Read the 9DOF sensor data
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    /* Read the current temperature from BNO055 */
    int8_t temp = bno.getTemp();

    // Read the BNO055 Quaternion
    imu::Quaternion quat = bno.getQuat();

    // Fill the ROS2 message for IMU data
    imu_msg.header.stamp.nanosec = (uint32_t)(millis() * 1e6);
    imu_msg.header.stamp.sec = (uint32_t)(millis() / 1000);
    imu_msg.header.frame_id.data = (char*)("imu_link");

    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();
    imu_msg.orientation.w = quat.w(); 

    imu_msg.angular_velocity.x = angVelocityData.gyro.x;
    imu_msg.angular_velocity.y = angVelocityData.gyro.y;
    imu_msg.angular_velocity.z = angVelocityData.gyro.z;

    imu_msg.linear_acceleration.x = linearAccelData.acceleration.x;
    imu_msg.linear_acceleration.y = linearAccelData.acceleration.y;
    imu_msg.linear_acceleration.z = linearAccelData.acceleration.z;

    // Publish the IMU message
    rcl_ret_t ret_imu_ok = rcl_publish(&imu_publisher, &imu_msg, NULL);

    // Display IMU data on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    // display.println(); // print blank line
    display.print("Temperature C: "); display.println(temp);
    // display the IMU Orientation
    // display.print("qW: "); display.println(quat.w(), 4);
    // display.print("qX: "); display.println(quat.x(), 4);
    // display.print("qY: "); display.println(quat.y(), 4);
    // display.print("qZ: "); display.println(quat.z(), 4);
    display.print("oX: "); display.print(orientationData.acceleration.heading, 1); display.print(" | ");
    display.print("oY: "); display.println(orientationData.acceleration.pitch, 1);
    display.print("oZ: "); display.println(orientationData.acceleration.roll, 1);
    display.print("gX: "); display.print(gravityData.gyro.x, 1); display.print(" | ");
    display.print("gY: "); display.println(gravityData.gyro.y, 1);
    display.print("gZ: "); display.println(gravityData.gyro.z, 1);
    // Display the calibration status (Sys | Gyro | Accel | Magno)
    display.println();
    display.print("C: S="); display.print(system); display.print("|"); 
    display.print("G="); display.print(gyro); display.print("|");
    display.print("A="); display.print(accel); display.print("|");
    display.print("M="); display.println(mag);
    // Populate the Oled Display
    display.display();

}

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    // Constants for motor speed calculation
    const float MAX_LINEAR_SPEED = 1.0; // m/s
    const float MAX_ANGULAR_SPEED = 1.0; // rad/s
    const int MAX_PWM = 255;

    // Linear and angular speeds from the message
    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;

    // Calculate motor speeds
    int left_speed = (int)((linear_x - angular_z) * (MAX_PWM / MAX_LINEAR_SPEED));
    int right_speed = (int)((linear_x + angular_z) * (MAX_PWM / MAX_LINEAR_SPEED));

    // Ensure the speed is within the PWM limits
    left_speed = constrain(left_speed, -MAX_PWM, MAX_PWM);
    right_speed = constrain(right_speed, -MAX_PWM, MAX_PWM);

    // Set motor speeds for all four motors
    set_motor_speed(0, left_speed);  // Left front motor
    set_motor_speed(1, right_speed); // Right front motor
    set_motor_speed(2, left_speed);  // Left rear motor
    set_motor_speed(3, right_speed); // Right rear motor

    // Flash the on-board LED if any motor is moving
    if (left_speed != 0 || right_speed != 0) {
        digitalWrite(LED, HIGH);
    } else {
        digitalWrite(LED, LOW);
    }
}

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);

    // Set Micro-ROS transport
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    // Create init_options and support
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create node
    rclc_node_init_default(&node, "esp32_node", "", &support);


    // Create IMU publisher
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");

    // Create encoder publisher
    rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "wheel_encoders");

    // Create subscriber
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // Initialize encoder message
    encoder_msg.data.size = 4;
    encoder_msg.data.capacity = 4;
    encoder_msg.data.data = (int32_t *)malloc(encoder_msg.data.capacity * sizeof(int32_t));

    // Create timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100), // Publish encoder data and IMU data every 100 ms
        timer_callback);

    // Create executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

    // Set up motors and encoders and IMU
    setup_motors();
    setup_encoders();
    setup_imu();
    setup_oled_display();

    // Set up LED pin
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(10); // Small delay to prevent overwhelming the CPU
}
