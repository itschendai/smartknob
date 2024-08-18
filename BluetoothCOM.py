import serial
import time
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from comtypes import CLSCTX_ALL
from ctypes import cast, POINTER
import pyautogui
import subprocess

illustrator_path = r'D:\Adobe\Adobe Illustrator 2024\Support Files\Contents\Windows\Illustrator.exe'

last_volume = 0

# Replace 'COMX' with your specific COM port (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
bluetooth_port = 'COM3'
baud_rate = 115200

def connect_to_bluetooth(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect: {e}")
        return None

def send_command(ser, command):
    if ser:
        ser.write(command.encode())  # Send the command as bytes
        ser.write(b'\n')  # Optional: Send a newline after the command
        #print(f"Sent: {command}")

def read_from_bluetooth(ser):
    #time.sleep(0.2)
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()  # Read and decode the response
            if response:
                print(f"Smart Knob: {response}")
            if response.startswith("Volume"):  # Check if the command starts with "Volume"
                    control_volume(response)
            elif response == "AltTab":  # Check for "AltTab" command
                alt_tab()
            elif response == "Illustrator":  # Check for "Illustrator" command
                launch_illustrator()

def alt_tab():
    pyautogui.keyDown('alt')  # Hold down the Alt key
    pyautogui.press('tab')    # Press the Tab key
    pyautogui.keyUp('alt')    # Release the Alt key
    print("Alt + Tab pressed.")

def launch_illustrator():
    try:
        subprocess.Popen(illustrator_path)  # Launch Adobe Illustrator
        print("Adobe Illustrator launched.")
    except FileNotFoundError:
        print(f"Illustrator not found at {illustrator_path}. Please check the path.")

def control_volume(command):
    global last_volume
    print("setting volume")
    try:
        # Extract the numeric part from the command (e.g., "Volume50" -> 50)
        volume_percentage = int(command.replace("Volume", ""))

        if volume_percentage != last_volume:
    
            # Ensure the percentage is between 0 and 100
            if 0 <= volume_percentage <= 100:
                devices = AudioUtilities.GetSpeakers()
                interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
                volume = cast(interface, POINTER(IAudioEndpointVolume))
                
                # Set the volume level (as a scalar value between 0.0 and 1.0)
                volume.SetMasterVolumeLevelScalar(volume_percentage / 100.0, None)
                print(f"Volume set to {volume_percentage}%")
                last_volume = volume
            else:
                print("Invalid volume percentage. Please use a value between 0 and 100.")
            
            
    
    except ValueError:
        print("Invalid command format. Please use 'VolumeX' where X is a number between 0 and 100.")

def main():
    ser = connect_to_bluetooth(bluetooth_port, baud_rate)

    if ser:
        time.sleep(2)  # Give some time for the connection to establish

        # Example: Interactive loop to send commands and receive responses
        try:
            while True:
                # command = input()
                # if command.lower() == 'exit':
                #     print("Exiting...")
                #     break
                # send_command(ser, command)
                read_from_bluetooth(ser)
        except KeyboardInterrupt:
            print("\nExiting...")

        ser.close()

if __name__ == "__main__":
    main()
