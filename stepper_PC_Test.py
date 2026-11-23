import serial
import time

PORT = "COM4"       # 장치 관리자에서 확인한 COM 번호로 변경
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=3)

time.sleep(1)

print("STM32 연결 완료")
print("방향,이동각도 입력 (최대 20도). 예: CW,10 / CCW,12.5")
print("종료: Q")

while True:
    command = input("명령: ").strip().upper()

    if command == "Q":
        break

    ser.write((command + "\n").encode())

    print("TX:", command)

    response = ser.readline().decode(errors="ignore").strip()
    if response == "STARTED":
        print("RX:", response)
        # Large moves can take longer than the initial response timeout.
        while response in ("STARTED", ""):
            response = ser.readline().decode(errors="ignore").strip()

    if response:
        print("RX:", response)
    else:
        print("RX: 응답 없음")

ser.close()
