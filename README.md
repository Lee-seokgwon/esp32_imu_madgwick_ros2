# 🚀 ESP32 IMU micro-ROS Node

ESP32 기반 IMU 센서 노드로, micro-ROS를 통해 ROS 2와 통신합니다.  
ROBOTIS 칼리브레이션 알고리즘과 Madgwick 필터를 적용하여 **정확하고 안정적인 IMU 데이터**를 제공합니다.

---

## 🌟 소개

이 프로젝트는 **ESP32 마이크로컨트롤러**와 **MPU9250/MPU6500 IMU 센서**를 사용하여  
고정밀 관성 측정 데이터를 **micro-ROS를 이용해 ROS 2 시스템에 제공하는 esp32코드**입니다.

### 주요 특징

- ⚡ **고성능 비동기 처리**: 센서 읽기(1kHz) + ROS 퍼블리시(100Hz) 분리
- 🎯 **정밀한 칼리브레이션**: ROBOTIS 검증 알고리즘으로 바이어스 자동 보정
- 🧭 **정확한 자세 추정**: Madgwick 필터 기반 쿼터니언 융합
- 🚀 **실시간 성능**: micro-ROS 최적화로 지연 시간 최소화
- 🔧 **플러그 앤 플레이**: 자동 칼리브레이션으로 간편한 설정

---

## 🛠️ 테스트 환경

### ✅ 하드웨어

| 항목 | 사양 |
|------|------|
| MCU | ESP32-WROOM-32D (ESP32 Dev Module) |
| IMU | MPU9250 또는 MPU6500 |
| 통신 | I2C (SDA: GPIO21, SCL: GPIO22) |


### ✅ 소프트웨어

- Arduino IDE
- micro-ROS
- ROS 2 (테스트 환경: Humble)

---

## ⚙️ 기능 설명

### 1. 고성능 비동기 데이터 처리

- 센서 루프: 최대 **1kHz**로 IMU 데이터 읽기 및 Madgwick 필터링
- ROS 루프: **100Hz**로 안정적인 토픽 퍼블리시
- 병목 해결: micro-ROS 통신 지연이 센서 처리에 영향을 주지 않음

---

### 2. ROBOTIS 칼리브레이션 시스템

- **자이로스코프**: 512 샘플 평균으로 정적 바이어스 계산 및 실시간 보정
- **가속도계**: 512 샘플 평균으로 오프셋 보정 (**Z축 중력 보존**)
- **자동 진행**: 시작 시 자동으로 칼리브레이션 수행

---

### 3. Madgwick 필터 융합

- **6축 IMU 데이터**(가속도 + 자이로)를 쿼터니언으로 융합
- **드리프트 보정 및 노이즈 필터링**
- 빠른 **역제곱근 알고리즘**으로 성능 최적화

---

### 4. 출력 데이터

퍼블리시되는 `sensor_msgs/Imu` 메시지:

```cpp
- header.stamp             // 마이크로초 정밀도 타임스탬프
- orientation              // Madgwick 필터 출력 쿼터니언 (w,x,y,z)
- angular_velocity         // 칼리브레이션된 각속도 (rad/s)
- linear_acceleration      // 칼리브레이션된 선형가속도 (m/s²)
- *_covariance             // 미리 설정된 공분산 행렬
```

---

### 📦 Dependencies

Arduino IDE: 라이브러리 매니저에서 MPU9250_asukiaaa (1.5.13) 설치

Micro ROS : 
``` bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
---

## ✅ 실행 방법

### 0. 회로 구성

아래 표를 참고하여 **ESP32와 IMU(MPU9250/MPU6500)**를 연결하세요.

| ESP32 핀 | IMU 핀 (MPU9250/6500) |
|----------|------------------------|
| GND      | ADO                    |
| GPIO21   | SDA                    |
| GPIO22   | SCL                    |
| GND      | GND                    |
| 3.3V     | VCC                    |

> 🔧 참고: ADO를 GND에 연결하면 IMU I2C 주소는 `0x68`이 됩니다.

---

### 1. Arduino IDE로 펌웨어 업로드

- PlatformIO 또는 Arduino IDE에서 예제 코드를 열고,
- ESP32 Dev Module 보드를 선택한 후,
- COM 포트를 설정하고 펌웨어를 업로드하세요.

---

### 2. micro-ROS 에이전트 실행

Linux PC에서 다음 명령어로 micro-ROS serial 에이전트를 실행하세요:

```bash
sudo chmod 666 /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
> 🔧 참고: 실행환경에따라 시리얼 포트는 다를수 있음.

### 3. 토픽 정상 발행 확인 (/imu)

```bash
ros2 topic list
```

---

## 👥 Contributors

| 이름           | GitHub                                  | 소속              | 기여 내용                                                |
|----------------|------------------------------------------|-------------------|-----------------------------------------------------------|
| Jaeyoun Jung   | [@jjletsgo](https://github.com/jjletsgo)             | 경북대학교 (SEE)   | micro-ROS 통합 및 IMU 데이터 처리 구현                    |
| Seokgwon Lee   | [@Lee-Seokgwon](https://github.com/Lee-Seokgwon)     | 경북대학교 (SEE)   | 비동기 처리 로직 설계 및 ROBOTIS 바이어스 보정 로직 통합 |

