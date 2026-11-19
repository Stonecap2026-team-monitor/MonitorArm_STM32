# Smart Monitor Arm STM32 Firmware

> Jetson과 STM32 간 UART 커스텀 프로토콜 및 FreeRTOS 기반 제어 구조를 적용한 스마트 모니터 암 STM32 펌웨어  
> Capstone Design Project · In Progress

---

## Development Status

### 2026년 11월 19일 실험 상태 저장

- 엔코더는 최근 3개 측정값의 이동 평균을 모터 제어에 제공하며, 읽기 오류가 발생하면 평균용 샘플을 초기화합니다.
- Theta1은 이동 종료 후 2초를 기다리고, 새 유효 평균값을 최대 4초 동안 기다립니다. 정상값이 준비되면 즉시 진행합니다.
- 위치 오차가 3°를 초과하면 최대 3회 보정합니다. 보정 횟수는 보정 후 측정에 성공한 뒤 증가합니다.
- `THETA1_MAX_CORRECTION_DEG`는 실험을 위해 **100°로 임시 변경**한 상태입니다. 이전 10° 설정으로 복원할 수 있도록 기록합니다.
- 스텝 모터의 가감속 제어와 기존 UART 통신 규격을 유지합니다.
- 반복 실험 중 I2C 타임아웃과 엔코더 측정 실패에 따른 시스템 락이 관찰됐으며, 근본 원인은 확정하지 않았습니다.

실험은 `pyserial` 설치 후 `python encoder_motion_logger.py --port COM4`로 실행합니다. 포트는 실제 연결에 맞게 지정합니다. `sim`은 보정된 영점 기준 `(20, -40, 30)`°, `zero`는 `(0, 0, 0)`°이며, `test`는 안내된 측정 순서를 실행합니다. 생성된 CSV는 로컬에 보관합니다.

`encoder_csv_logger.py`는 과거 진단 메시지 `0x87`용 도구입니다. 현재 펌웨어 실험에는 `encoder_motion_logger.py`를 사용합니다.

아래는 초기 개발 시점의 구현 현황입니다.

본 저장소는 **2026년 8월 6일 기준으로 개발 중인 캡스톤 프로젝트의 STM32 Firmware Snapshot**입니다.

전체 스마트 모니터 암 시스템은 아직 개발 중이지만, 포트폴리오에 작성한 다음 통신 기능은 코드 구현과 기본 검증을 완료했습니다.

### 구현 및 기본 검증 완료

- Jetson–STM32 UART 커스텀 Frame 설계
- 가변 길이 데이터 송수신 구조
- 상태 머신 기반 Byte 단위 수신 Parser
- START / END Byte 검증
- XOR Checksum 생성 및 검증
- Heartbeat 수신 및 ACK 응답
- 비정상 DATA LENGTH 검출
- 불완전한 Frame 수신 Timeout 복구
- 잘못된 Frame 이후 정상 Frame 재수신
- Big-Endian 기반 목표 각도 데이터 변환
- FreeRTOS Queue 기반 Communication Task–Motor Task 연동
- 현재 엔코더 각도 주기 송신
- Motor 동작 완료 및 오류 상태 송신
- Python 기반 PC–STM32 UART 통신 테스트

### 현재 개발 중

- Jetson 영상 처리 프로그램과 STM32의 실제 통합
- 전체 모터 제어부와의 통합 검증
- 통신 단절 시 모터 안전 정지
- SET_HOME 및 MOVE_HOME 기능 연동
- ACK Timeout 및 재전송 처리
- 전체 하드웨어 장시간 동작 시험

---

## Project Overview

본 프로젝트는 카메라를 통해 사용자의 자세를 분석하고, 사용자에게 적합한 위치로 모니터 암을 자동 조절하는 시스템입니다.

Jetson은 영상 분석 결과를 바탕으로 모터의 목표 각도를 계산하여 STM32로 전달합니다. STM32는 수신한 목표값에 따라 모터를 제어하고, 현재 각도와 동작 완료 및 오류 상태를 Jetson으로 회신합니다.

Jetson과 STM32 사이에서 명령과 상태 데이터를 구분하고 통신 오류를 검출하기 위해 UART 기반 커스텀 프로토콜을 설계했습니다.

고정 길이 데이터만 수신하는 방식이 아니라 메시지 종류에 따라 DATA 길이를 변경할 수 있도록 가변 길이 Frame을 정의했습니다. 또한 Checksum, Heartbeat, ACK 및 Timeout 처리를 적용하여 비정상 Frame을 검출하고 다음 정상 Frame을 다시 수신할 수 있도록 구성했습니다.

---

## My Role

본 프로젝트에서 **STM32 MCU 펌웨어 전체 개발**을 담당하고 있습니다.

통신 부분에서는 다음 기능을 직접 설계하고 구현했습니다.

- Jetson–STM32 통신 요구사항 정의
- UART Custom Protocol Frame 설계
- RX / TX Message ID 정의
- Frame 생성 및 Decode 함수 구현
- XOR Checksum 알고리즘 구현
- 상태 머신 기반 가변 길이 수신 Parser 구현
- Heartbeat 및 ACK 처리
- Frame 수신 Timeout 및 Parser Reset 처리
- Big-Endian 데이터 직렬화 및 역직렬화
- Communication Task 설계
- FreeRTOS Queue 기반 Motor Task 연동
- ChatGPT를 활용한 PC–STM32 UART 통신 테스트 코드 구성 및 동작 검증
- 오류 Frame 및 복구 동작 테스트

---

## Development Environment

| Item | Specification |
|---|---|
| MCU | STM32F446RE |
| IDE | STM32CubeIDE |
| Configuration | STM32CubeMX |
| RTOS | FreeRTOS / CMSIS-RTOS2 |
| Language | C |
| Communication | UART |
| Baud Rate | 115200 bps |
| Upper Controller | Jetson |
| Current Test Device | Windows PC |
| PC Test Language | Python |
| Encoder | AS5600 |
| Motor Interface | Stepper Motor / Servo Motor |

---

### Command Flow

```text
Jetson
  → UART Custom Frame
  → Communication Task
  → Frame 검증 및 Data Decode
  → motorCommandQueue
  → Motor Task
```

### Status Flow

```text
Motor Task
  → motorStatusQueue
  → Communication Task
  → UART Custom Frame 생성
  → Jetson
```

현재 엔코더 각도는 Communication Task에서 주기적으로 확인한 후 `CURRENT_ANGLE` Frame으로 Jetson에 송신합니다.

---

# UART Custom Protocol

## Protocol Design

Jetson과 STM32는 동일한 Frame 형식을 사용하여 데이터를 송수신합니다.

```text
+-------+--------+--------+----------+----------+------+
| START | MSG_ID | LENGTH |   DATA   | CHECKSUM | END  |
+-------+--------+--------+----------+----------+------+
| 1Byte | 1Byte  | 1Byte  | N Bytes  | 1Byte    |1Byte |
+-------+--------+--------+----------+----------+------+
```

| Field | Size | Description |
|---|---:|---|
| START | 1 Byte | Frame 시작 값 `0xAA` |
| MSG_ID | 1 Byte | 명령 또는 상태 메시지 식별 |
| LENGTH | 1 Byte | DATA 영역의 길이 |
| DATA | N Bytes | 명령값 또는 상태값 |
| CHECKSUM | 1 Byte | MSG_ID, LENGTH, DATA의 XOR 결과 |
| END | 1 Byte | Frame 종료 값 `0x55` |

START와 END Byte를 이용하여 Frame의 경계를 구분하고, LENGTH 값을 이용하여 가변 길이 DATA를 처리합니다.

Checksum은 `MSG_ID`, `LENGTH`, `DATA`를 XOR 연산하여 계산합니다. 수신한 Checksum과 STM32에서 다시 계산한 값이 일치하지 않으면 해당 Frame을 비정상 데이터로 판단하여 폐기합니다.

```text
CHECKSUM = MSG_ID XOR LENGTH XOR DATA[0] XOR ... XOR DATA[N-1]
```

현재 최대 DATA 길이는 16 Byte로 제한했습니다.

```c
#define PROTOCOL_START_BYTE      0xAAU
#define PROTOCOL_END_BYTE        0x55U
#define PROTOCOL_MAX_DATA_LENGTH 16U
#define PROTOCOL_OVERHEAD_LENGTH 5U
```

---

## Message IDs

### Jetson → STM32

| MSG_ID | Message | DATA | Current Status |
|---:|---|---|---|
| `0x01` | HEARTBEAT | Alive Counter, 1 Byte | 구현 및 기본 검증 완료 |
| `0x10` | SET_TARGET | θ1, θ2, θ3, 총 6 Byte | 구현 및 Queue 연동 |
| `0x11` | SET_HOME | Home θ1, θ2, θ3 | Message ID 정의 |
| `0x12` | MOVE_HOME | DATA 없음 | Message ID 정의 |

### STM32 → Jetson

| MSG_ID | Message | DATA | Current Status |
|---:|---|---|---|
| `0x80` | ACK | 수신한 MSG_ID, 1 Byte | 구현 및 기본 검증 완료 |
| `0x81` | STATUS | 시스템 상태 | Message ID 정의 |
| `0x82` | MOVE_DONE | 현재 구현본은 DATA 없음 | 구현 |
| `0x83` | ERROR | 현재 구현본은 DATA 없음 | 구현 |
| `0x84` | CURRENT_ANGLE | 현재 각도 × 10, 2 Byte | 구현 |

`SET_HOME`, `MOVE_HOME`, `STATUS`는 Protocol Message ID를 정의했으며 실제 시스템 기능과의 연동은 개발 중입니다.

`MOVE_DONE`과 `ERROR`는 현재 DATA가 없는 상태 Frame으로 구현했으며, 향후 최종 각도와 세부 오류 코드를 DATA에 추가할 예정입니다.

---

## Target Angle Data Format

Jetson은 3개의 목표 각도를 각각 0.1도 단위의 Signed 16-bit 정수로 변환하여 전송합니다.

```text
θ1 : 2 Bytes
θ2 : 2 Bytes
θ3 : 2 Bytes

Total DATA Length : 6 Bytes
```

예를 들어 목표 각도가 다음과 같을 경우,

```text
θ1 = 10.5°
θ2 = -5.0°
θ3 = 20.0°
```

전송되는 정수값은 다음과 같습니다.

```text
θ1_x10 = 105
θ2_x10 = -50
θ3_x10 = 200
```

각 데이터는 Big-Endian 형식으로 전송합니다.

```text
High Byte → Low Byte
```

STM32는 수신한 6 Byte DATA를 각각 `int16_t` 값으로 복원한 후 `motorCommandQueue`를 통해 Motor Task에 전달합니다.

```text
DATA[0], DATA[1] → θ1
DATA[2], DATA[3] → θ2
DATA[4], DATA[5] → θ3
```

---

## Current Angle Data Format

STM32는 엔코더에서 측정한 각도를 0.1도 단위의 Signed 16-bit 정수로 변환하여 송신합니다.

```text
Actual Angle  : 35.7°
Transmit Data : 357
```

현재 구현본에서는 1개의 현재 각도를 2 Byte Big-Endian 형식으로 송신합니다.

```text
CURRENT_ANGLE DATA Length : 2 Bytes
```

현재 각도는 200 ms 주기로 Jetson에 송신합니다.

```c
#define CURRENT_ANGLE_TX_PERIOD_MS 200U
```

---

# RX State Machine

## Byte-by-Byte Parsing

초기에는 Heartbeat Frame의 길이에 맞춰 UART 데이터를 6 Byte 고정 길이로 수신했습니다.

하지만 Message마다 DATA 길이가 다르기 때문에 고정 길이 수신 방식으로는 `SET_TARGET`, `MOVE_HOME` 등 서로 다른 Frame을 처리하기 어려웠습니다.

이를 해결하기 위해 UART 데이터를 1 Byte씩 수신하고, 각 Byte를 상태 머신 Parser에 전달하도록 구조를 변경했습니다.

```text
WAIT_START
    ↓
READ_MSG_ID
    ↓
READ_LENGTH
    ↓
READ_DATA
    ↓
READ_CHECKSUM
    ↓
READ_END
    ↓
FRAME_COMPLETE
```

### Parser States

| State | Description |
|---|---|
| `WAIT_START` | START Byte `0xAA` 대기 |
| `READ_MSG_ID` | Message ID 저장 |
| `READ_LENGTH` | DATA 길이 확인 |
| `READ_DATA` | LENGTH만큼 DATA 저장 |
| `READ_CHECKSUM` | 수신 Checksum 저장 |
| `READ_END` | END Byte `0x55` 확인 |

### Parsing Process

1. UART에서 1 Byte 수신
2. 수신 Byte를 `Protocol_RxProcessByte()`에 전달
3. START Byte 확인
4. MSG_ID 저장
5. DATA LENGTH 확인
6. LENGTH만큼 DATA 저장
7. Checksum 저장
8. END Byte 확인
9. `frame_complete` Flag 설정
10. 완성된 Frame Decode
11. Checksum 및 Frame 형식 검증
12. 정상 Message만 Communication Task에서 처리

```c
Protocol_RxProcessByte(&rxParser, rxByte);
```

Frame 처리가 완료되거나 오류가 발생하면 Parser 내부 상태와 DATA Buffer를 초기화합니다.

---

## Parser Data Structure

```c
typedef struct
{
    ProtocolRxState_t state;
    uint8_t msg_id;
    uint8_t data_length;
    uint8_t data_index;
    uint8_t data[PROTOCOL_MAX_DATA_LENGTH];
    uint8_t checksum;
    uint8_t frame_complete;
} ProtocolRxParser_t;
```

Parser 구조체 내부에 현재 수신 State, Message ID, DATA 길이, DATA Buffer, Checksum 및 Frame 완료 상태를 저장합니다.

이를 통해 UART 데이터가 여러 번에 나누어 수신되더라도 이전 수신 상태를 유지하면서 다음 Byte를 이어서 처리할 수 있습니다.

---

# Communication Task

## Main Responsibilities

Communication Task는 다음 기능을 담당합니다.

- UART Byte 단위 수신
- Frame Parser 실행
- Frame Decode 및 Checksum 검증
- HEARTBEAT Message 처리
- ACK Frame 생성 및 송신
- SET_TARGET Data 변환
- Motor Command Queue 전달
- Current Angle 주기 송신
- Motor Status Queue 확인
- MOVE_DONE / ERROR Frame 송신
- Frame 수신 Timeout 감시
- Heartbeat Timeout 감시

---

## Heartbeat and ACK

Jetson은 1초마다 Alive Counter를 포함한 HEARTBEAT Frame을 송신합니다.

```text
MSG_ID : 0x01
LENGTH : 1 Byte
DATA   : Alive Counter
```

STM32가 정상적인 HEARTBEAT Frame을 수신하면 다음 값을 갱신합니다.

- 최근 Alive Counter
- Heartbeat 수신 횟수
- 마지막 Heartbeat 수신 시간
- 통신 연결 상태

이후 수신한 Message ID를 DATA에 포함한 ACK Frame을 Jetson으로 송신합니다.

```text
HEARTBEAT RX

AA 01 01 [ALIVE_COUNTER] [CHECKSUM] 55
```

```text
ACK TX

AA 80 01 01 [CHECKSUM] 55
```

ACK의 DATA `0x01`은 STM32가 HEARTBEAT Message ID `0x01`을 정상적으로 수신했음을 의미합니다.

---

## SET_TARGET Processing

Communication Task가 `SET_TARGET` Message를 수신하면 DATA 길이가 6 Byte인지 확인합니다.

정상적인 Frame이면 3개의 목표 각도를 Big-Endian `int16_t` 값으로 복원합니다.

```c
motorCommand.theta1_x10 =
    Protocol_ReadInt16BigEndian(&rxMessage.data[0]);

motorCommand.theta2_x10 =
    Protocol_ReadInt16BigEndian(&rxMessage.data[2]);

motorCommand.theta3_x10 =
    Protocol_ReadInt16BigEndian(&rxMessage.data[4]);
```

변환된 목표 각도는 `motorCommandQueue`를 통해 Motor Task로 전달합니다.

```text
Communication Task
        │
        │ motorCommandQueue
        │ θ1 / θ2 / θ3 Target Angle
        ▼
Motor Task
```

Queue 전달에 성공한 경우 STM32는 `SET_TARGET` Message에 대한 ACK를 송신합니다.

---

## Motor Status Processing

Motor Task에서 동작 완료 또는 오류가 발생하면 `motorStatusQueue`를 통해 Communication Task로 상태를 전달합니다.

```text
Motor Task
        │
        │ motorStatusQueue
        │ MOVE_DONE / ERROR
        ▼
Communication Task
```

Communication Task는 수신한 상태에 따라 다음 Frame을 생성하여 Jetson으로 송신합니다.

```text
MOTOR_STATUS_MOVE_DONE → MSG_ID 0x82
MOTOR_STATUS_ERROR     → MSG_ID 0x83
```

---

# Communication Error Handling

## 1. START / END Byte Validation

수신 Frame의 첫 번째 Byte가 `0xAA`인지 확인하고, 마지막 Byte가 `0x55`인지 확인합니다.

값이 올바르지 않으면 해당 Frame을 폐기합니다.

---

## 2. DATA Length Validation

수신한 LENGTH가 `PROTOCOL_MAX_DATA_LENGTH`보다 큰 경우 잘못된 Frame으로 판단합니다.

```c
if (parser->data_length > PROTOCOL_MAX_DATA_LENGTH)
{
    Protocol_RxParserReset(parser);
}
```

Parser를 초기화한 후 다음 START Byte부터 새로운 Frame 수신을 시작합니다.

---

## 3. Checksum Validation

수신한 Checksum과 STM32에서 다시 계산한 Checksum을 비교합니다.

값이 일치하지 않으면 `PROTOCOL_ERROR_CHECKSUM`을 반환하고 해당 Message를 처리하지 않습니다.

---

## 4. Incomplete Frame Timeout

Frame 수신이 시작된 후 일정 시간 동안 다음 Byte가 들어오지 않으면 불완전한 Frame으로 판단합니다.

```c
#define PROTOCOL_RX_TIMEOUT_MS 20U
```

20 ms 동안 Frame 수신이 완료되지 않으면 Parser를 초기화합니다.

```text
Incomplete Frame
      ↓
20 ms Timeout
      ↓
Parser Reset
      ↓
Next START Byte Wait
```

이 구조를 통해 Frame 수신 도중 Jetson 프로그램이 종료되거나 데이터가 중간에 끊겨도 다음 정상 Frame을 다시 수신할 수 있습니다.

---

## 5. Heartbeat Timeout

일정 시간 동안 HEARTBEAT Message가 수신되지 않으면 Jetson과의 통신이 끊어진 것으로 판단합니다.

```c
#define HEARTBEAT_TIMEOUT_MS 3000U
```

3초 이상 Heartbeat가 수신되지 않으면 다음 상태를 설정합니다.

```c
communicationLost = 1U;
```

현재 구현본에서는 통신 단절 상태를 검출하며, 통신 단절 시 Motor 안전 정지 기능과의 연동은 개발 중입니다.

---

# FreeRTOS Queue Integration

통신 처리와 모터 제어 기능을 하나의 Task에서 모두 실행하지 않고, 각 기능을 독립적인 Task로 분리했습니다.

### Communication Task → Motor Task

```text
motorCommandQueue
```

전달 데이터:

- Command Type
- θ1 Target Angle
- θ2 Target Angle
- θ3 Target Angle

### Motor Task → Communication Task

```text
motorStatusQueue
```

전달 데이터:

- MOVE_DONE
- ERROR

Queue를 이용하여 Communication Task가 Motor Task의 내부 동작에 직접 의존하지 않도록 구성했습니다.

이를 통해 UART 통신 처리와 실제 모터 제어 기능을 분리하고, 각 Task가 자신의 역할에 집중할 수 있도록 설계했습니다.

---

# PC Communication Test

실제 Jetson 프로그램과 통합하기 전, Python과 USB-UART를 이용하여 PC–STM32 통신 기능을 검증했습니다.

> PC 통신 검증에 사용한 `serialtest.py`는 ChatGPT를 활용하여 생성한 테스트 코드입니다.  
> 본 코드는 최종 Jetson 프로그램이 아니라, STM32에 구현한 커스텀 프로토콜의 송수신 및 오류 처리 동작을 확인하기 위한 테스트 목적으로 사용했습니다.

```text

PC Python Test Program
        ↕ UART
STM32 Communication Task

`serialtest.py`는 다음 기능을 수행합니다.

- HEARTBEAT Frame 1초 주기 송신
- Alive Counter 증가
- SET_TARGET Frame 송신
- 3개의 목표 각도 Big-Endian 변환
- Frame Checksum 생성
- ACK Frame 수신
- CURRENT_ANGLE Frame 수신
- MOVE_DONE Frame 수신
- ERROR Frame 수신
- 수신 Checksum 검증

테스트 환경에 따라 COM Port를 변경해야 합니다.

```python
PORT = "COM9"
BAUD = 115200
```

Python 실행에는 `pyserial`이 필요합니다.

```bash
pip install pyserial
python serialtest.py
```

---

# Troubleshooting

## UART 가변 길이 Frame Parsing 오류

### Problem

UART 데이터를 상태 머신 Parser로 처리하는 과정에서 정상적인 Frame을 송신했지만 Parsing 오류가 발생했습니다.

### Analysis

UART Baud Rate는 115200 bps로 설정했습니다.

Start Bit 1개, Data Bit 8개, Stop Bit 1개를 사용하는 경우 1 Byte 전송에는 총 10 Bit가 필요합니다.

```text
1 Byte Transmission Time
= 10 bits / 115200 bps
≈ 86.8 μs
```

초기 수신 반복문에는 1 ms 지연이 포함되어 있었습니다.

```text
UART 1 Byte 전송 시간 : 약 86.8 μs
수신 반복문 지연      : 1 ms
```

수신 처리 지연이 Byte 전송 간격보다 길어 연속적으로 들어오는 데이터를 정상적인 시점에 처리하지 못한다고 판단했습니다.

### Solution

- 수신 반복문의 불필요한 1 ms 지연 제거
- UART 데이터를 1 Byte씩 수신
- 수신 Byte를 즉시 상태 머신 Parser에 전달
- 잘못된 LENGTH 수신 시 Parser 전체 초기화
- 불완전한 Frame Timeout 처리 추가
- Frame 완료 후 Parser Buffer 초기화

### Result

다음 동작을 확인했습니다.

- 가변 길이 Frame 정상 수신
- HEARTBEAT 수신
- ACK Frame 응답
- SET_TARGET 6 Byte DATA 수신
- 잘못된 Checksum Frame 검출
- 최대 길이를 초과한 LENGTH 검출
- 잘못된 LENGTH 이후 정상 Frame 재수신
- 불완전한 Frame Timeout 이후 정상 복구

---

# Test Results

| Test Item | Result |
|---|---|
| HEARTBEAT Frame 수신 | PASS |
| ACK Frame 송신 | PASS |
| 가변 길이 DATA 수신 | PASS |
| SET_TARGET 6 Byte Parsing | Implemented |
| XOR Checksum 검증 | PASS |
| Invalid Checksum 검출 | PASS |
| Invalid LENGTH 검출 | PASS |
| Invalid LENGTH 이후 정상 복구 | PASS |
| Incomplete Frame Timeout 복구 | PASS |
| Current Angle 주기 송신 | Implemented |
| Motor Command Queue 전달 | Implemented |
| Motor Status Queue 수신 | Implemented |
| Jetson 실제 프로그램 통합 | In Progress |
| 전체 Motor System 통합 | In Progress |

`PASS`는 PC–STM32 환경에서 동작을 확인한 항목이며, `Implemented`는 코드 구현 후 전체 시스템과의 추가 통합 검증이 필요한 항목입니다.

---

# Main Source Files

| File | Description |
|---|---|
| `protocol.c/.h` | Frame 생성, Decode, Checksum 및 RX State Machine Parser |
| `communicationTask.c/.h` | UART 송수신, Heartbeat, ACK, Timeout 및 Queue 연동 |
| `motorCommand.h` | Motor Command 및 Status 데이터 정의 |
| `motorTask.c/.h` | 목표 각도 기반 Motor 제어 Task |
| `encoderTask.c/.h` | Encoder 측정 Task |
| `as5600.c/.h` | AS5600 I2C 통신 및 현재 각도 측정 |
| `stepper.c/.h` | Stepper Motor 제어 |
| `servo.c/.h` | Servo Motor 제어 |
| `serialtest.py` | ChatGPT를 활용해 생성한 PC 기반 UART Protocol 테스트 코드 |

---

# Project Structure

```text
Smart-Monitor-Arm-STM32-Firmware/
├── Core/
│   ├── Inc/
│   │   ├── protocol.h
│   │   ├── communicationTask.h
│   │   ├── motorCommand.h
│   │   ├── motorTask.h
│   │   ├── encoderTask.h
│   │   ├── as5600.h
│   │   ├── stepper.h
│   │   └── servo.h
│   │
│   └── Src/
│       ├── protocol.c
│       ├── communicationTask.c
│       ├── motorTask.c
│       ├── encoderTask.c
│       ├── as5600.c
│       ├── stepper.c
│       └── servo.c
│
├── Drivers/
├── Middlewares/
├── serialtest.py
├── capstone.ioc
└── README.md
```

---

# Current Limitations

현재 버전에는 다음 기능이 포함되지 않았거나 추가적인 검증이 필요합니다.

- Jetson 영상 처리 프로그램과 STM32의 실제 통합
- 전체 3축 Motor 제어 검증
- ACK 미수신 시 자동 재전송
- NACK Message 처리
- Sequence Number 기반 중복 Frame 검출
- XOR Checksum의 CRC 전환
- 통신 단절 시 Motor 안전 정지
- SET_HOME 기능 연동
- MOVE_HOME 기능 연동
- STATUS DATA 형식 확정
- MOVE_DONE 최종 각도 DATA 추가
- ERROR 세부 오류 코드 추가
- UART Interrupt 또는 DMA 수신 방식 적용
- 전체 시스템 장시간 통신 안정성 시험

---

# Future Work

- Jetson–STM32 실제 통합 송수신
- 모터 제어 시스템 전체 연동
- 통신 단절 시 안전 정지 구현
- ACK Timeout 및 재전송 구조 추가
- NACK 및 오류 코드 세분화
- Sequence Number 추가
- CRC 기반 오류 검출 검토
- UART Interrupt / DMA 수신 방식 검토
- 통신 상태 Log 저장
- 장시간 반복 통신 시험
- 전체 스마트 모니터 암 동작 영상 추가

---

# What I Learned

본 프로젝트를 통해 UART 통신은 단순히 데이터를 송수신하는 것만으로 끝나는 것이 아니라는 점을 배웠습니다.

메시지 종류와 데이터 길이를 정의하고, Frame의 시작과 끝을 구분하며, 수신 오류와 통신 단절 상황까지 고려해야 안정적인 시스템을 구성할 수 있음을 경험했습니다.

또한 고정 길이 수신 방식에서 가변 길이 상태 머신 Parser 구조로 변경하면서, 요구사항에 따라 확장할 수 있는 Protocol 구조를 설계하는 방법을 익혔습니다.

잘못된 Checksum, 비정상 LENGTH 및 불완전한 Frame을 직접 입력해 본 뒤 정상 Frame으로 복구되는지 확인하며 오류 검출뿐만 아니라 오류 이후의 복구 과정도 중요하다는 점을 배웠습니다.

FreeRTOS Queue를 이용해 Communication Task와 Motor Task를 분리하면서, Task 간 책임을 구분하고 데이터 흐름을 명확하게 설계하는 경험을 쌓았습니다.

---

# Notice

본 저장소는 국립한밭대학교 전자공학과 캡스톤디자인에서 진행 중인 프로젝트입니다.

**2026년 8월 6일 기준의 개발 Snapshot이며, 전체 스마트 모니터 암 시스템의 최종 결과물이 아닙니다.**

포트폴리오에 기재된 다음 기능은 구현되어 있습니다.

- UART Custom Frame 설계
- 상태 머신 기반 가변 길이 Parser
- XOR Checksum 검증
- HEARTBEAT 및 ACK
- DATA LENGTH 오류 검출
- Frame 수신 Timeout
- 비정상 Frame 이후 Parser 복구
- FreeRTOS Queue 기반 Communication Task–Motor Task 연동

Jetson 영상 처리 프로그램, 전체 모터 제어부 및 하드웨어 시스템과의 통합은 계속 진행 중입니다.

---

