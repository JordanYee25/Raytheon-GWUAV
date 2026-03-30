import os
import importlib.util
import sys
import subprocess

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"

# need SERIAL FOR c2
# Need 3.9 or below python
# check usb.

'''
ivanwinters@Ivans-MacBook-Pro ~ % ioreg -p IOUSB
+-o Root  <class IORegistryEntry, id 0x100000100, retain 35>
  +-o AppleT6000USBXHCI@01000000  <class AppleT6000USBXHCI, id 0x100000543, regis$
  +-o AppleT6000USBXHCI@02000000  <class AppleT6000USBXHCI, id 0x100000474, regis$
  +-o AppleT6000USBXHCI@00000000  <class AppleT6000USBXHCI, id 0x100000380, regis$
    +-o HDMI USB Camera @00100000  <class IOUSBHostDevice, id 0x10039795a, regist$
ivanwinters@Ivans-MacBook-Pro ~ % 
'''

files_to_check = [
    'aruco_library.py',      
    'calibration_logi.npz',
    'challenge1.py',
    'Logs/',                 
    'preflight-c1.py'
]


def check_camera():
    print("\nCamera check:")
    desired_camera_id = "32e4:3415"  # camera id 1bcf:2c99(mac) linux - 32e4:3415
    try:
        # Run the lsusb command and capture the output
        result = subprocess.run(['lsusb'], stdout=subprocess.PIPE, text=True)
        lsusb_output = result.stdout

        # Check if the desired camera ID is in the output
        for line in lsusb_output.splitlines():
            if desired_camera_id in line:
                # Extract the bus and device number from the line
                parts = line.split()
                bus = parts[1]
                device = parts[3].strip(':')
                print(f"{GREEN}✓{RESET} Camera with ID {desired_camera_id} is connected on Bus {bus}, Device {device}.")
                return True

        # If the camera ID is not found
        print(f"{RED}✗{RESET} Camera with ID {desired_camera_id} is not connected.")
        return False
    except Exception as e:
        print(f"{RED}✗{RESET} Failed to check camera: {e}")
        return False


# Check that Python 3.9 or below is installed
def check_python_version():
    version = sys.version_info
    if version.major != 3 or version.minor > 9:
        print(f"{RED}✗{RESET} P2ython 3.9 or below is required (detected {version.major}.{version.minor}.{version.micro}).")
        return False
    else:
        print(f"{GREEN}✓{RESET} Python {version.major}.{version.minor}.{version.micro} is acceptable.")
        return True


def check_files():
    print("File checks:")
    all_exist = True
    for file in files_to_check:
        if os.path.exists(file):
            print(f"{GREEN}✓{RESET} {file} exists.")
        else:
            print(f"{RED}✗{RESET} {file} does not exist.")
            all_exist = False
    return all_exist

def check_packages():
    print("\nPackage checks:")
    packages_to_check = [
        'numpy',
        'cv2',
        'dronekit',  # for Ardupilot communication
        'pymavlink', # MAVLink support
        # ASS NEEDED FOR Challenge 2
        # 'Jetson.GPIO',  # Jetson-specific GPIO control or 
        # 'RPi.GPIO'     # OR this. THIS IS FOR RPi
    ]
    all_installed = True
    for package in packages_to_check:
        if importlib.util.find_spec(package) is not None:
            print(f"{GREEN}✓{RESET} Package '{package}' is installed.")
        else:
            print(f"{RED}✗{RESET} Package '{package}' is missing.")
            all_installed = False
    return all_installed

def check_permissions():
    print("\nPermission checks:")
    devices = {
        'Flight Controller': '/dev/ttyACM0',
        'Telemetry': '/dev/ttyUSB0'
    }
    all_permissions_ok = True

    for device_name, device_path in devices.items():
        try:
            mode = os.stat(device_path).st_mode
            if oct(mode)[-3:] == '666':
                print(f"{GREEN}✓{RESET} {device_name} ({device_path}) has correct permissions (666).")
            else:
                print(f"{RED}✗{RESET} {device_name} ({device_path}) does not have correct permissions (expected 666, found {oct(mode)[-3:]}).")
                all_permissions_ok = False
        except FileNotFoundError:
            print(f"{RED}✗{RESET} {device_name} ({device_path}) does not exist.")
            all_permissions_ok = False

    return all_permissions_ok
def main():
    version_ok = check_python_version()
    files_ok = check_files()
    packages_ok = check_packages()
    permissions_ok = check_permissions()
    camera_ok = check_camera()
    if version_ok and files_ok and packages_ok and permissions_ok and camera_ok:
        print(f"\n{GREEN}All pre-flight checks passed. Ready to run challenges!{RESET}")
    else:
        print(f"\n{RED}Pre-flight checks failed.{RESET}")

if __name__ == '__main__':
    main()