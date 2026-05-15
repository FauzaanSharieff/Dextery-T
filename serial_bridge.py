import serial
import time

# Change these to your actual COM ports
CONTROLLER_PORT = "COM3"
ROBOT_PORT = "COM4"

BAUD_RATE = 115200

VALID_COMMANDS = {
    "+x", "-x",
    "+y", "-y",
    "+z", "-z",
    "idle,o", "idle,c"
}

def open_serial(port_name):
    return serial.Serial(
        port=port_name,
        baudrate=BAUD_RATE,
        timeout=0.01
    )

def main():
    controller = open_serial(CONTROLLER_PORT)
    robot = open_serial(ROBOT_PORT)

    # Arduinos usually reset when serial connection opens
    time.sleep(2)

    print("Dextery-T PySerial bridge started.")
    print(f"Controller port: {CONTROLLER_PORT}")
    print(f"Robot port:      {ROBOT_PORT}")
    print("Forwarding valid commands...\n")

    last_command = None

    while True:
        # Read command from controller
        if controller.in_waiting > 0:
            raw_data = controller.readline()
            command = raw_data.decode(errors="ignore").strip().lower()

            if command in VALID_COMMANDS:
                # Forward command to robot
                robot.write((command + "\n").encode())

                # Only print when the command changes, to avoid spam
                if command != last_command:
                    print(f"Controller -> Robot: {command}")
                    last_command = command

            elif command != "":
                print(f"Ignored invalid command: {command}")

        # Optional: read messages coming back from robot
        if robot.in_waiting > 0:
            raw_response = robot.readline()
            response = raw_response.decode(errors="ignore").strip()

            if response != "":
                print(f"Robot -> Laptop: {response}")

        time.sleep(0.002)

if __name__ == "__main__":
    main()