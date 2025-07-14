class KalmanFilter {
public:
    float Q; // 과정 잡음 공분산
    float R; // 측정 잡음 공분산
    float A; // 상태 전이 행렬
    float B; // 제어 입력 효과 행렬
    float C; // 측정 행렬

    float x; // 추정된 상태
    float cov; // 추정된 공분산

    KalmanFilter() : Q(0.1), R(5.0), A(1), B(0), C(1), x(0), cov(1) {}

    void setMeasurementNoise(float noise) {
        R = noise;
    }

    void setProcessNoise(float noise) {
        Q = noise;
    }

    void update(float measurement) {
        // 예측 단계
        float predX = A * x + B;
        float predCov = A * cov * A + Q;

        // 칼만 이득 계산
        float K = predCov * C / (C * predCov * C + R);

        // 업데이트 단계
        x = predX + K * (measurement - C * predX);
        cov = predCov - K * C * predCov;
    }

    float getValue() {
        return x;
    }
};

#include "MPU9250.h"
#include <Wire.h>
#include "MAX30105.h"
#include "WiFi101.h"
#include <SPI.h>

MAX30105 particleSensor;
MPU9250 mpu;
KalmanFilter kalmanX, kalmanY, kalmanZ;

char ssid[] = "SK_WiFiGIGAC324_2.4G";    // Wi-Fi SSID
char pass[] = "1809054299"; // Wi-Fi Password
int status = WL_IDLE_STATUS;

WiFiServer server(80);

void setup() {
  WiFi.setPins(8, 7, 4, 2);  // Wi-Fi 모듈의 핀 설정
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing...");

  // I2C 통신 속도 낮추기
  Wire.begin();
  Wire.setClock(I2C_SPEED_FAST);

  // MAX30105 센서 초기화
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Could not find a valid MAX30105 sensor, check wiring!");
    while (1);
  } else {
    Serial.println("MAX30105 sensor found.");
  }

  // MAX30105 센서 설정
  byte ledBrightness = 0x28; // 0=Off to 255=50mA
  byte sampleAverage = 8; // 1, 2, 4, 8, 16, 32
  byte ledMode = 2; // 1 = Red only, 2 = Red + IR
  int sampleRate = 1600; // 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; // 69, 118, 215, 411
  int adcRange = 4096; // 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  Serial.println("Sensor setup complete.");

  if (!mpu.setup(0x68)) {
    Serial.println("MPU connection failed. Please check your connection.");
    while (1) delay(5000);
  }
  
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
  
  mpu.setAccBias(-0.16657, -1.25652, -0.11697);  // 가속도 바이어스 값 (g 단위로 수정)
  mpu.setGyroBias(-9.68, 1.66, 4.95);  // 자이로 바이어스 값 (deg/s 단위)
  mpu.setMagBias(96.64, 392.68, 271.00);  // 자력계 바이어스 값 (mG 단위)
  mpu.setMagScale(2.64, 0.65, 0.93);  // 자력계 스케일 값

  // Kalman 필터 초기화
  kalmanX.setProcessNoise(0.1);
  kalmanX.setMeasurementNoise(5.0);

  kalmanY.setProcessNoise(0.1);
  kalmanY.setMeasurementNoise(5.0);

  kalmanZ.setProcessNoise(0.1);
  kalmanZ.setMeasurementNoise(5.0);

  // Wi-Fi 네트워크에 연결
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  Serial.println("Connected to Wi-Fi");
  server.begin();
  Serial.println("Server started");

  // IP 주소 출력
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void loop() {
  // 클라이언트 연결 확인
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");

    // 클라이언트가 연결된 동안 루프
    while (client.connected()) {
      if (particleSensor.check()) {
        // 센서 데이터 읽기
        uint32_t redValue = particleSensor.getRed();

        // 클라이언트로 데이터 전송
        client.print(-redValue);
        client.print(",");

        // 디버그: 시리얼 모니터에 데이터 출력
        Serial.print("Red: ");
        Serial.print(-redValue);
        Serial.print(",");
      } else {
        Serial.println("No data available from sensor.");
      }

      if (mpu.update()) {
        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();

        kalmanX.update(ax);
        kalmanY.update(ay);
        kalmanZ.update(az);

        client.print(kalmanX.getValue());
        client.print(",");
        client.print(kalmanY.getValue());
        client.print(",");
        client.print(kalmanZ.getValue());
        client.println();

        // 디버그: 시리얼 모니터에 데이터 출력
        Serial.print(kalmanX.getValue());
        Serial.print(",");
        Serial.print(kalmanY.getValue());
        Serial.print(",");
        Serial.print(kalmanZ.getValue());
        Serial.println();
      }
      
    }

    client.stop();
    Serial.println("Client disconnected");
  }
}
