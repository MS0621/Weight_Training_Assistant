# 웨어러블 심박수 기반 웨이트 트레이닝 보조 시스템

## 1. 프로젝트 개요

초심자들이 세트 간 적절한 **휴식 시간**을 설정하는 데 어려움을 겪는 문제를 해결하기 위해, 본 프로젝트는 **실시간 심박수**를 기반으로 개인 맞춤형 휴식 가이드를 제공하는 **웨어러블 장치**를 개발하였습니다.

- 기존에는 감각이나 고정된 시간(예: 3분)에 의존
- 본 시스템은 **심박수 기준의 자동화된 휴식 종료** 기능을 통해 효율적인 트레이닝을 유도

---

## 2. ⚙시스템 구성

### 개발환경

| 구성 요소               | 사용 도구 / 플랫폼      |
|------------------------|--------------------------|
| Feather M0 WiFi 보드   | Arduino IDE              |
| PPG & ACC signal processing  | MATLAB                   |
| LSTM 모델 훈련         | Google Colab (Python, TensorFlow) |



### 사용 부품

| 구성 요소 | 설명 |
|-----------|------|
| Feather M0 WiFi | 저전력 마이크로컨트롤러, WiFi 통신 지원 |
| MAX30102 | PPG 기반 심박수 및 SpO2 측정 센서 |
| MPU9250 | 9축 IMU (가속도, 자이로, 자기장) 센서 |
| 배터리 파트 | 3.3V 리튬 전지 |

- 통신 방식: I2C
- 전원: 3.3V~5V
- 전체 구성은 회로도와 하드웨어 연결로 구현됨
- 시스템의 구성도는 다음과 같다.
<img width="1700" height="532" alt="Image" src="https://github.com/user-attachments/assets/ac8a2ce8-dee2-4e90-9251-5c94afbe779e" />


- 시스템을 착용한 모습은 다음과 같다.
<img width="1223" height="607" alt="image" src="https://github.com/user-attachments/assets/4d1cce33-6448-45a6-ae71-fc7c32f992b5" />


---

## 3. 실험 및 데이터 수집

### 실험 조건

- 대상자: 초심자 2명 (A, B), 중상급자 2명 (C, D)
- 종목: 벤치프레스 (4세트, 10RM 기준)
- 휴식 기준: 심박수 103 BPM 도달 시까지
- 총 데이터 수: 6,148개

### 수집된 데이터

| 항목 | 설명 |
|------|------|
| 시간(sec) | 초 단위 시간 정보 |
| Heart_rate | 심박수 |
| State | 운동 중: 1, 휴식 중: 0 |
| 심박의 변이 | 직전 값 대비 차이 |
| id | 사용자 식별자 |
| RPE | 자각 운동 강도 (Rate of Perceived Exertion) |

---

## 4. 모델 구성 및 학습

### 사용 모델: LSTM

LSTM 모델 구조



| 항목 | 내용 |
|------|------|
| 모델 구조 | LSTM 기반 시계열 예측 모델 |
| 입력 | 시간에 따른 심박수, 상태, 변이량 |
| 출력 | 휴식 종료 시점의 예상 심박수 |
| 성능 지표 | MSE: 1.112 / MAE: 0.928 / R²: 0.984 |

- 심박수의 흐름을 학습하여 다음 운동 시작 타이밍을 예측
- 실제 측정값과의 비교 결과 정확도 우수

---

## 5. 결과

### 장치 구현

- 실착용 가능한 형태로 하드웨어 완성
- 실시간 심박수 측정 가능
<img width="500" height="500" alt="Image" src="https://github.com/user-attachments/assets/631d1e81-233d-4048-9758-3841ffa4264f" />

- 삼성 갤럭시 워치 4와 비교 측정 시 RMSE: 1.38 BPM

### 예측 성능 시각화

- 훈련 손실 감소 그래프 확인
- 실제 심박수와 예측값이 유사하게 추종함

<img width="872" height="309" alt="image" src="https://github.com/user-attachments/assets/9cde22a5-2d0b-472c-b23c-e1173e9b336c" />


---

## 6. 결론

- 심박수 기반 맞춤형 **휴식 시간 가이드 시스템** 개발 완료
- LSTM 모델을 통해 사용자 개인 회복 특성에 따른 **스마트 피드백 제공**
- 운동의 효율 향상 및 초심자 트레이닝 지원 효과 기대

---

## 7. 향후 개선 방향

- 동작 중 발생하는 **동잡음** 제거 성능 강화 필요
- **주파수 영역 분석** 기반 필터링 기술 도입 예정
- **데이터 다양성 확대**: 연령, 성별, 체력 등
- 심박 외 **SpO2, 호흡률, 피로도** 등 복합 생체신호 도입 예정
- **휴식 종료 알림** 기능 (진동, 사운드) 탑재 고려

---

## 8. 학술제 포스터
<img width="7015" height="9933" alt="Image" src="https://github.com/user-attachments/assets/9df7f9cf-fe71-4a1c-9bfb-281aab074726" />

---

## 9. More about project
[▶️ PDF로 확인하기 (Google Drive)](https://drive.google.com/file/d/1IqVn2Y9rs6z56KnE-h0iZyrhdn9PIOGp/preview)

---
