import serial
import struct
import threading
import time


# ============================================================
# SERIAL CONFIG
# ============================================================

PORT = "COM4"
BAUD = 115200

START = 0xAA
END = 0x55
MAX_DATA_LENGTH = 16

# Heartbeat ACK가 계속 출력되어 입력창이 지저분해지는 것을 방지
# 확인하고 싶으면 True로 변경
SHOW_HEARTBEAT_ACK = False

# Set True to display received angle telemetry.
SHOW_ANGLES = False

# Recovery timeout
CLEAR_ERROR_TIMEOUT_SEC = 5.0
MOVE_HOME_TIMEOUT_SEC = 90.0


# ============================================================
# Python -> STM32
# ============================================================

MSG_HEARTBEAT = 0x01

MSG_SET_TARGET = 0x10
MSG_SET_HOME = 0x11
MSG_MOVE_HOME = 0x12
MSG_JOG = 0x13
MSG_CLEAR_ERROR = 0x14


# ============================================================
# STM32 -> Python
# ============================================================

MSG_ACK = 0x80
MSG_STATUS = 0x81
MSG_COMMAND_DONE = 0x82
MSG_ERROR = 0x83
MSG_CURRENT_ANGLE = 0x84
MSG_CURRENT_COMMAND_ANGLES = 0x85
MSG_READY = 0x86


# ============================================================
# JOG AXIS
# ============================================================

AXIS_THETA1 = 1
AXIS_THETA2 = 2
AXIS_THETA3 = 3


# ============================================================
# COMMAND / ERROR NAME
# ============================================================

COMMAND_NAMES = {
    MSG_HEARTBEAT: "HEARTBEAT",
    MSG_SET_TARGET: "SET_TARGET",
    MSG_SET_HOME: "SET_HOME",
    MSG_MOVE_HOME: "MOVE_HOME",
    MSG_JOG: "JOG",
    MSG_CLEAR_ERROR: "CLEAR_ERROR",
}


ERROR_NAMES = {
    0x00: "NONE",
    0x01: "INVALID_TARGET",
    0x02: "INVALID_AXIS",
    0x03: "ANGLE_LIMIT",
    0x04: "STEPPER_BUSY",
    0x05: "STEPPER",
    0x06: "SERVO2",
    0x07: "SERVO3",
    0x08: "HOME_NOT_SET",
    0x09: "INIT",
    0x0A: "QUEUE",
    0x0B: "INVALID_COMMAND",
    0x0C: "COMM_LOST",
    0x0D: "ENCODER",
    0x0E: "SYSTEM_LOCKED",
    0x0F: "ESTOP",
}


# ============================================================
# GLOBAL
# ============================================================

alive_counter = 0
heartbeat_enabled = True

write_lock = threading.Lock()

# Recovery용 이벤트
clear_error_done_event = threading.Event()
move_home_done_event = threading.Event()
recovery_error_event = threading.Event()

last_error_code = None


# ============================================================
# CHECKSUM
# ============================================================

def checksum(msg_id, data):
    cs = msg_id ^ len(data)

    for byte in data:
        cs ^= byte

    return cs & 0xFF


# ============================================================
# FRAME BUILD
# ============================================================

def build_frame(msg_id, data):
    frame = bytearray()

    frame.append(START)
    frame.append(msg_id)
    frame.append(len(data))

    frame.extend(data)

    frame.append(
        checksum(
            msg_id,
            data
        )
    )

    frame.append(END)

    return frame


# ============================================================
# SAFE UART WRITE
# ============================================================

def safe_write(ser, frame):
    with write_lock:
        ser.write(frame)
        ser.flush()


# ============================================================
# HEARTBEAT THREAD
# ============================================================

def heartbeat_thread(ser):
    global alive_counter
    global heartbeat_enabled

    while ser.is_open:

        if heartbeat_enabled:
            alive_counter = (
                alive_counter + 1
            ) & 0xFF

            data = bytes(
                [alive_counter]
            )

            frame = build_frame(
                MSG_HEARTBEAT,
                data
            )

            try:
                safe_write(
                    ser,
                    frame
                )

            except serial.SerialException:
                break

        time.sleep(1.0)


# ============================================================
# SET_TARGET
# ============================================================

def send_set_target(
    ser,
    theta1_deg,
    theta2_deg,
    theta3_deg
):
    theta1_x10 = int(
        round(theta1_deg * 10.0)
    )

    theta2_x10 = int(
        round(theta2_deg * 10.0)
    )

    theta3_x10 = int(
        round(theta3_deg * 10.0)
    )

    data = struct.pack(
        ">hhh",
        theta1_x10,
        theta2_x10,
        theta3_x10
    )

    frame = build_frame(
        MSG_SET_TARGET,
        data
    )

    safe_write(
        ser,
        frame
    )

    print()
    print(
        f"SET_TARGET TX : "
        f"theta1={theta1_deg:+.1f} deg, "
        f"theta2={theta2_deg:+.1f} deg, "
        f"theta3={theta3_deg:+.1f} deg"
    )

    print(
        "TX :",
        frame.hex(" ").upper()
    )


# ============================================================
# JOG
# ============================================================

def send_jog(
    ser,
    axis,
    delta_deg
):
    delta_x10 = int(
        round(delta_deg * 10.0)
    )

    data = bytearray()

    data.append(axis)

    data.extend(
        struct.pack(
            ">h",
            delta_x10
        )
    )

    frame = build_frame(
        MSG_JOG,
        data
    )

    safe_write(
        ser,
        frame
    )

    print()
    print(
        f"JOG TX : "
        f"axis={axis}, "
        f"delta={delta_deg:+.1f} deg"
    )

    print(
        "TX :",
        frame.hex(" ").upper()
    )


# ============================================================
# SET_HOME
# ============================================================

def send_set_home(ser):
    data = b""

    frame = build_frame(
        MSG_SET_HOME,
        data
    )

    safe_write(
        ser,
        frame
    )

    print()
    print("SET_HOME TX")

    print(
        "TX :",
        frame.hex(" ").upper()
    )


# ============================================================
# MOVE_HOME
# ============================================================

def send_move_home(ser):
    data = b""

    frame = build_frame(
        MSG_MOVE_HOME,
        data
    )

    safe_write(
        ser,
        frame
    )

    print()
    print("MOVE_HOME TX")

    print(
        "TX :",
        frame.hex(" ").upper()
    )


# ============================================================
# CLEAR_ERROR
# ============================================================

def send_clear_error(ser):
    data = b""

    frame = build_frame(
        MSG_CLEAR_ERROR,
        data
    )

    safe_write(
        ser,
        frame
    )

    print()
    print("CLEAR_ERROR TX")

    print(
        "TX :",
        frame.hex(" ").upper()
    )


# ============================================================
# WAIT COMMAND DONE OR ERROR
# ============================================================

def wait_for_done_or_error(
    done_event,
    timeout_sec
):
    start_time = time.monotonic()

    while (
        time.monotonic() - start_time
    ) < timeout_sec:

        if done_event.is_set():
            return True

        if recovery_error_event.is_set():
            return False

        time.sleep(0.05)

    return False


# ============================================================
# RECOVER -> HOME
# ============================================================

def recover_to_home(ser):
    global last_error_code

    print()
    print(
        "========== RECOVERY START =========="
    )

    # --------------------------------------------------------
    # 이전 이벤트 초기화
    # --------------------------------------------------------

    clear_error_done_event.clear()
    move_home_done_event.clear()
    recovery_error_event.clear()

    last_error_code = None


    # --------------------------------------------------------
    # 1. CLEAR_ERROR
    # --------------------------------------------------------

    print()
    print(
        "[1/2] CLEAR_ERROR 요청"
    )

    send_clear_error(
        ser
    )

    clear_success = wait_for_done_or_error(
        clear_error_done_event,
        CLEAR_ERROR_TIMEOUT_SEC
    )

    if not clear_success:

        if recovery_error_event.is_set():

            error_name = ERROR_NAMES.get(
                last_error_code,
                f"UNKNOWN(0x{last_error_code:02X})"
                if last_error_code is not None
                else "UNKNOWN"
            )

            print()
            print(
                "RECOVERY 중단"
            )

            print(
                f"CLEAR_ERROR 실패 : "
                f"{error_name}"
            )

        else:

            print()
            print(
                "CLEAR_ERROR TIMEOUT"
            )

        print(
            "MOVE_HOME을 실행하지 않습니다."
        )

        return


    print()
    print(
        "CLEAR_ERROR 완료"
    )


    # --------------------------------------------------------
    # 2. MOVE_HOME
    # --------------------------------------------------------

    move_home_done_event.clear()
    recovery_error_event.clear()

    last_error_code = None

    print()
    print(
        "[2/2] MOVE_HOME 요청"
    )

    send_move_home(
        ser
    )

    move_success = wait_for_done_or_error(
        move_home_done_event,
        MOVE_HOME_TIMEOUT_SEC
    )

    if not move_success:

        if recovery_error_event.is_set():

            error_name = ERROR_NAMES.get(
                last_error_code,
                f"UNKNOWN(0x{last_error_code:02X})"
                if last_error_code is not None
                else "UNKNOWN"
            )

            print()
            print(
                "RECOVERY 중단"
            )

            print(
                f"MOVE_HOME 실패 : "
                f"{error_name}"
            )

        else:

            print()
            print(
                "MOVE_HOME TIMEOUT"
            )

        return


    print()
    print(
        "MOVE_HOME 완료"
    )

    print()
    print(
        "========== RECOVERY COMPLETE =========="
    )


# ============================================================
# RX THREAD
# ============================================================

def receive_thread(ser):
    global last_error_code

    try:

        while ser.is_open:

            # ------------------------------------------------
            # START
            # ------------------------------------------------

            start = ser.read(1)

            if len(start) == 0:
                continue

            if start != bytes([START]):
                continue


            # ------------------------------------------------
            # MSG_ID + LENGTH
            # ------------------------------------------------

            header = ser.read(2)

            if len(header) != 2:
                print(
                    "\nRX HEADER LENGTH ERROR"
                )

                continue

            msg_id = header[0]
            length = header[1]

            if length > MAX_DATA_LENGTH:
                print(
                    f"\nRX DATA LENGTH INVALID : "
                    f"{length}"
                )

                continue


            # ------------------------------------------------
            # DATA
            # ------------------------------------------------

            data = ser.read(length)

            if len(data) != length:
                print(
                    f"\nDATA LENGTH ERROR : "
                    f"expected={length}, "
                    f"received={len(data)}"
                )

                continue


            # ------------------------------------------------
            # CHECKSUM
            # ------------------------------------------------

            rx_checksum = ser.read(1)

            if len(rx_checksum) != 1:
                print(
                    "\nCHECKSUM BYTE ERROR"
                )

                continue


            # ------------------------------------------------
            # END
            # ------------------------------------------------

            end = ser.read(1)

            if end != bytes([END]):
                print(
                    "\nEND BYTE ERROR"
                )

                continue


            # ------------------------------------------------
            # CHECKSUM VERIFY
            # ------------------------------------------------

            calc_checksum = checksum(
                msg_id,
                data
            )

            if calc_checksum != rx_checksum[0]:

                print(
                    f"\nCHECKSUM ERROR : "
                    f"RX=0x{rx_checksum[0]:02X}, "
                    f"CALC=0x{calc_checksum:02X}"
                )

                continue


            # =================================================
            # ACK
            # =================================================

            if msg_id == MSG_ACK:

                if len(data) != 1:
                    print(
                        "\nACK LENGTH ERROR"
                    )

                    continue

                ack_id = data[0]

                if ack_id == MSG_HEARTBEAT:

                    if SHOW_HEARTBEAT_ACK:
                        print(
                            "\nACK : "
                            "HEARTBEAT (0x01)"
                        )

                else:

                    command_name = COMMAND_NAMES.get(
                        ack_id,
                        f"UNKNOWN(0x{ack_id:02X})"
                    )

                    print(
                        f"\nACK : "
                        f"{command_name} "
                        f"(0x{ack_id:02X})"
                    )


            # =================================================
            # COMMAND DONE
            # =================================================

            elif msg_id == MSG_COMMAND_DONE:

                if len(data) != 1:
                    print(
                        f"\nCOMMAND_DONE LENGTH ERROR : "
                        f"{len(data)}"
                    )

                    continue

                completed_id = data[0]

                command_name = COMMAND_NAMES.get(
                    completed_id,
                    f"UNKNOWN(0x{completed_id:02X})"
                )

                print(
                    f"\nCOMMAND DONE : "
                    f"{command_name} "
                    f"(0x{completed_id:02X})"
                )


                # ---------------------------------------------
                # Recovery Event
                # ---------------------------------------------

                if completed_id == MSG_CLEAR_ERROR:

                    clear_error_done_event.set()


                elif completed_id == MSG_MOVE_HOME:

                    move_home_done_event.set()


            # =================================================
            # ERROR
            # =================================================

            elif msg_id == MSG_ERROR:

                if len(data) != 1:

                    print(
                        f"\nMOTOR ERROR : "
                        f"INVALID DATA LENGTH "
                        f"({len(data)})"
                    )

                    continue

                error_code = data[0]

                last_error_code = error_code

                error_name = ERROR_NAMES.get(
                    error_code,
                    f"UNKNOWN(0x{error_code:02X})"
                )

                print(
                    f"\nMOTOR ERROR : "
                    f"{error_name} "
                    f"(0x{error_code:02X})"
                )


                # Recovery가 진행 중이라면
                # 바로 중단할 수 있도록 알림
                recovery_error_event.set()


            # =================================================
            # AS5600 ACTUAL THETA1
            # =================================================

            elif msg_id == MSG_CURRENT_ANGLE:

                if SHOW_ANGLES:
                    print(
                        f"\n0x84 RX RAW : "
                        f"len={len(data)}, "
                        f"data={data.hex(' ').upper()}"
                    )

                if len(data) == 2:

                    angle_x10 = struct.unpack(
                        ">h",
                        data
                    )[0]

                    angle_deg = (
                        angle_x10 / 10.0
                    )

                    if SHOW_ANGLES:
                        print(
                            f"ENCODER ANGLE : "
                            f"{angle_deg:+.1f} deg"
                        )

                else:

                    print(
                        f"0x84 LENGTH ERROR : "
                        f"{len(data)}"
                    )


            # =================================================
            # CURRENT COMMAND ANGLES
            # =================================================

            elif msg_id == MSG_CURRENT_COMMAND_ANGLES:

                if SHOW_ANGLES:
                    print(
                        f"\n0x85 RX RAW : "
                        f"len={len(data)}, "
                        f"data={data.hex(' ').upper()}"
                    )

                if len(data) == 6:

                    (
                        theta1_x10,
                        theta2_x10,
                        theta3_x10
                    ) = struct.unpack(
                        ">hhh",
                        data
                    )

                    theta1 = (
                        theta1_x10 / 10.0
                    )

                    theta2 = (
                        theta2_x10 / 10.0
                    )

                    theta3 = (
                        theta3_x10 / 10.0
                    )

                    if SHOW_ANGLES:
                        print(
                            f"CURRENT COMMAND ANGLES : "
                            f"theta1={theta1:+.1f} deg, "
                            f"theta2={theta2:+.1f} deg, "
                            f"theta3={theta3:+.1f} deg"
                        )

                else:

                    print(
                        f"0x85 LENGTH ERROR : "
                        f"{len(data)}"
                    )


            # =================================================
            # STATUS
            # =================================================

            elif msg_id == MSG_STATUS:

                print(
                    "\nSTATUS :",
                    data.hex(" ").upper()
                )


            # =================================================
            # READY
            # =================================================

            elif msg_id == MSG_READY:

                if len(data) != 0:

                    print(
                        f"\nREADY LENGTH ERROR : "
                        f"{len(data)}"
                    )

                    continue

                print(
                    "\nSTM32 READY"
                )


            # =================================================
            # UNKNOWN
            # =================================================

            else:

                print(
                    f"\nUNKNOWN RX : "
                    f"MSG=0x{msg_id:02X}, "
                    f"LEN={length}, "
                    f"DATA={data.hex(' ').upper()}"
                )


    except serial.SerialException as exc:

        print(
            f"\nRX Serial Error : {exc}"
        )


# ============================================================
# MENU
# ============================================================

def print_menu():

    print()

    print(
        "========== MOTOR TEST =========="
    )

    print(
        "t  : SET_TARGET"
    )

    print()

    print(
        "1+ : theta1 JOG +1 deg"
    )

    print(
        "1- : theta1 JOG -1 deg"
    )

    print(
        "2+ : theta2 JOG +1 deg"
    )

    print(
        "2- : theta2 JOG -1 deg"
    )

    print(
        "3+ : theta3 JOG +1 deg"
    )

    print(
        "3- : theta3 JOG -1 deg"
    )

    print()

    print(
        "h  : SET_HOME"
    )

    print(
        "m  : MOVE_HOME"
    )

    print()

    print(
        "r  : CLEAR_ERROR"
    )

    print(
        "rh : CLEAR_ERROR -> MOVE_HOME"
    )

    print()

    print(
        "x  : HEARTBEAT OFF"
    )

    print(
        "c  : HEARTBEAT ON"
    )

    print()

    print(
        "q  : QUIT"
    )

    print(
        "================================"
    )


# ============================================================
# MAIN
# ============================================================

def main():

    global heartbeat_enabled

    try:

        with serial.Serial(
            PORT,
            BAUD,
            timeout=0.1,
            write_timeout=1.0
        ) as ser:

            print(
                f"Serial Open : "
                f"{PORT} / {BAUD}"
            )

            time.sleep(2.0)

            ser.reset_input_buffer()
            ser.reset_output_buffer()


            # =================================================
            # RX THREAD
            # =================================================

            rx_thread = threading.Thread(
                target=receive_thread,
                args=(ser,),
                daemon=True
            )

            rx_thread.start()


            # =================================================
            # HEARTBEAT THREAD
            # =================================================

            hb_thread = threading.Thread(
                target=heartbeat_thread,
                args=(ser,),
                daemon=True
            )

            hb_thread.start()


            print_menu()


            # =================================================
            # COMMAND LOOP
            # =================================================

            while True:

                command = input(
                    "\n> "
                ).strip().lower()


                # ---------------------------------------------
                # SET_TARGET
                # ---------------------------------------------

                if command == "t":

                    try:

                        theta1 = float(
                            input(
                                "theta1 target : "
                            )
                        )

                        theta2 = float(
                            input(
                                "theta2 target : "
                            )
                        )

                        theta3 = float(
                            input(
                                "theta3 target : "
                            )
                        )

                    except ValueError:

                        print(
                            "숫자를 입력해야 합니다."
                        )

                        continue


                    send_set_target(
                        ser,
                        theta1,
                        theta2,
                        theta3
                    )


                # ---------------------------------------------
                # THETA1 JOG
                # ---------------------------------------------

                elif command == "1+":

                    send_jog(
                        ser,
                        AXIS_THETA1,
                        +1.0
                    )


                elif command == "1-":

                    send_jog(
                        ser,
                        AXIS_THETA1,
                        -1.0
                    )


                # ---------------------------------------------
                # THETA2 JOG
                # ---------------------------------------------

                elif command == "2+":

                    send_jog(
                        ser,
                        AXIS_THETA2,
                        +1.0
                    )


                elif command == "2-":

                    send_jog(
                        ser,
                        AXIS_THETA2,
                        -1.0
                    )


                # ---------------------------------------------
                # THETA3 JOG
                # ---------------------------------------------

                elif command == "3+":

                    send_jog(
                        ser,
                        AXIS_THETA3,
                        +1.0
                    )


                elif command == "3-":

                    send_jog(
                        ser,
                        AXIS_THETA3,
                        -1.0
                    )


                # ---------------------------------------------
                # SET_HOME
                # ---------------------------------------------

                elif command == "h":

                    send_set_home(
                        ser
                    )


                # ---------------------------------------------
                # MOVE_HOME
                # ---------------------------------------------

                elif command == "m":

                    send_move_home(
                        ser
                    )


                # ---------------------------------------------
                # CLEAR_ERROR
                # ---------------------------------------------

                elif command == "r":

                    send_clear_error(
                        ser
                    )


                # ---------------------------------------------
                # RECOVERY -> HOME
                # ---------------------------------------------

                elif command == "rh":

                    recover_to_home(
                        ser
                    )


                # ---------------------------------------------
                # HEARTBEAT OFF
                # ---------------------------------------------

                elif command == "x":

                    heartbeat_enabled = False

                    print()
                    print(
                        "HEARTBEAT OFF"
                    )

                    print(
                        "STM32 heartbeat timeout을 기다리세요."
                    )


                # ---------------------------------------------
                # HEARTBEAT ON
                # ---------------------------------------------

                elif command == "c":

                    heartbeat_enabled = True

                    print()
                    print(
                        "HEARTBEAT ON"
                    )


                # ---------------------------------------------
                # QUIT
                # ---------------------------------------------

                elif command == "q":

                    print(
                        "Exit"
                    )

                    break


                # ---------------------------------------------
                # MENU
                # ---------------------------------------------

                elif command == "help":

                    print_menu()


                # ---------------------------------------------
                # UNKNOWN
                # ---------------------------------------------

                else:

                    print(
                        "Unknown command"
                    )

                    print_menu()


    except serial.SerialException as exc:

        print(
            f"Serial Error : {exc}"
        )


    except KeyboardInterrupt:

        print(
            "\nStopped"
        )


# ============================================================
# PROGRAM START
# ============================================================

if __name__ == "__main__":
    main()