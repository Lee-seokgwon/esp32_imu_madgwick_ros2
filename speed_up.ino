//esp32 wroom 32d (esp32 dev module), mpu 9250/6500
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <MPU9250_asukiaaa.h>

// Madgwick filter
class MadgwickFilter {
private:
    float beta;
    float q0, q1, q2, q3;
    float invSampleFreq;
    
    // 역제곱근
    inline float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        y = y * (1.5f - (halfx * y * y));
        return y; 
    }
    
public:
    MadgwickFilter() : beta(0.1f), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}
    
    void begin(float sampleFrequency) {
        invSampleFreq = 1.0f / sampleFrequency;
    }
    
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533f;
        gy *= 0.0174533f;
        gz *= 0.0174533f;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion
        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
    
    // Get quaternion components
    float getQ0() { return q0; }
    float getQ1() { return q1; }
    float getQ2() { return q2; }
    float getQ3() { return q3; }
};

// micro-ROS error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// I2C pins for ESP32
#ifdef ESP32
#define SDA_PIN 21
#define SCL_PIN 22
#endif

// micro-ROS objects
rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// IMU sensor and filter objects
MPU9250_asukiaaa mySensor;
MadgwickFilter filter;

// 센서 데이터 캐시 (여러 번 읽기 방지)
struct SensorData {
    float aX, aY, aZ;
    float gX, gY, gZ;
    bool valid;
} sensorData;

// Constants
#define IMU_PUBLISH_FREQUENCY 100 // 100Hz
#define GRAVITY 9.80665f
#define DEG_TO_RAD_CONST 0.0174533f

// 미리 계산된 covariance 행렬
static const float orientation_cov[9] = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
static const float angular_vel_cov[9] = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
static const float linear_acc_cov[9] = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

void error_loop() {
    while(1) {
        delay(100);
    }
}

bool readSensorData() {
    if (mySensor.accelUpdate() != 0 || mySensor.gyroUpdate() != 0) {
        sensorData.valid = false;
        return false;
    }
    
    sensorData.aX = mySensor.accelX();
    sensorData.aY = mySensor.accelY();
    sensorData.aZ = mySensor.accelZ();
    sensorData.gX = mySensor.gyroX();
    sensorData.gY = mySensor.gyroY();
    sensorData.gZ = mySensor.gyroZ();
    sensorData.valid = true;
    
    return true;
}

void updateIMU() {
    if (!readSensorData()) return;

    filter.updateIMU(sensorData.gX, sensorData.gY, sensorData.gZ, 
                     sensorData.aX, sensorData.aY, sensorData.aZ);
}

// 저주파수로 ROS 메시지 퍼블리시
void publishIMU() {
    unsigned long start_time = micros();
    
    if (!sensorData.valid) return;
    
    // IMU 메시지 채우기 (캐시된 데이터 사용)
    imu_msg.linear_acceleration.x = sensorData.aX * GRAVITY;
    imu_msg.linear_acceleration.y = sensorData.aY * GRAVITY;
    imu_msg.linear_acceleration.z = sensorData.aZ * GRAVITY;
    
    imu_msg.angular_velocity.x = sensorData.gX * DEG_TO_RAD_CONST;
    imu_msg.angular_velocity.y = sensorData.gY * DEG_TO_RAD_CONST;
    imu_msg.angular_velocity.z = sensorData.gZ * DEG_TO_RAD_CONST;
    
    // orientation (madgwick output. Quaternion)
    imu_msg.orientation.w = filter.getQ0();
    imu_msg.orientation.x = filter.getQ1();
    imu_msg.orientation.y = filter.getQ2();
    imu_msg.orientation.z = filter.getQ3();
    
    unsigned long current_micros = micros();
    imu_msg.header.stamp.sec = current_micros / 1000000UL;
    imu_msg.header.stamp.nanosec = (current_micros % 1000000UL) * 1000UL;
    
    // 퍼블리시
    rcl_ret_t ret = rcl_publish(&publisher, &imu_msg, NULL);
}

void setup() {
    Serial.begin(115200);
    set_microros_transports();
    delay(2000);
    
    // I2C 최적화 설정
    #ifdef ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // 400kHz
    Wire.setTimeOut(1000); // 1ms timeout
    mySensor.setWire(&Wire);
    #else
    Wire.begin();
    Wire.setClock(400000);
    mySensor.setWire(&Wire);
    #endif
    
    // IMU 센서 초기화
    mySensor.beginAccel();
    mySensor.beginGyro();
    
    // Madgwick 필터 초기화
    filter.begin(IMU_PUBLISH_FREQUENCY);
    
    // micro-ROS 초기화
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_imu_node", "", &support));   
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"));    
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    
    // 메시지 초기화
    imu_msg.header.frame_id.data = (char*)malloc(20);
    strcpy(imu_msg.header.frame_id.data, "imu_link");
    imu_msg.header.frame_id.size = strlen("imu_link");
    imu_msg.header.frame_id.capacity = 20;
    
    // Covariance 미리 설정
    memcpy(imu_msg.orientation_covariance, orientation_cov, sizeof(orientation_cov));
    memcpy(imu_msg.angular_velocity_covariance, angular_vel_cov, sizeof(angular_vel_cov));
    memcpy(imu_msg.linear_acceleration_covariance, linear_acc_cov, sizeof(linear_acc_cov));
}

// IMU -> ASAP , Micro_ROS(bottle neck) -> slow down enough
// Asyncronous task -> imu speed up!
void loop() {
    static unsigned long last_publish = 0;
    static unsigned long last_imu_update = 0;
    unsigned long current_time = micros();
    
    // 고주파수 IMU 업데이트 (가능한 한 자주)
    if (current_time - last_imu_update >= 1000) { // ~1kHz
        updateIMU();
        last_imu_update = current_time;
    }
    
    // 저주파수 퍼블리시 (100Hz)
    if (current_time - last_publish >= 10000) { // 10ms = 100Hz
        publishIMU();
        last_publish = current_time;
    }
    
    // micro-ROS executor (짧은 timeout)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
}
