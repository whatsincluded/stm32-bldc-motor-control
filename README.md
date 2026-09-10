# STM32G431 BLDC Motor Control

B-G431B-ESC1 보드를 이용한 BLDC 모터 제어 프로젝트

## 목표

- STM32 레지스터 기반 펌웨어 구현
- Hall Sensor 기반 6-Step Commutation
- TIM1 Complementary PWM 구현
- Dead-time 및 보호 로직 구현
- 모터 속도 제어

## Hardware

- MCU: STM32G431CBU6
- Board: B-G431B-ESC1
- Motor: BL42S-24026N
- Hall Sensor: Hall A/B/C

## 개발 진행 상황

- [x] 개발 환경 구축
- [x] 시스템 클럭 설정
- [x] Hall Sensor 하드웨어 연결 및 입력 확인
- [ ] TIM1 PWM 출력
- [ ] Hall Sensor 상태 순서 확인
- [ ] 6-Step Commutation
- [ ] 모터 구동
- [ ] 속도 제어

## Debugging

- [Hall Sensor 연결부 디버깅](Docs/debugging/hall_sensor_connection_issue.md)