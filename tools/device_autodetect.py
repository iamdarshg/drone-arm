#!/usr/bin/env python3
import argparse
import os
import shutil
import sys
import time
import subprocess

# RP2350 USB IDs
RP2350_VID = 0x2E8A
RP2350_PID = 0x000F # Bootloader PID

def find_rp2350_mount():
    """Find the mount point of an RP2350 device in BOOTSEL mode."""
    # This is a simplified version that looks for 'RPI-RP2' or 'RP2350' in mount points
    # On Linux, they usually mount under /media/$USER/
    if sys.platform == "linux":
        try:
            output = subprocess.check_output(['lsblk', '-no', 'MOUNTPOINT,LABEL']).decode()
            for line in output.splitlines():
                if 'RP2350' in line or 'RPI-RP2' in line:
                    parts = line.split()
                    if parts:
                        return parts[0]
        except Exception:
            pass
    return None

def read_metadata(mount_point):
    """Read INFO_UF2.TXT from the device."""
    info_path = os.path.join(mount_point, 'INFO_UF2.TXT')
    if os.path.exists(info_path):
        with open(info_path, 'r') as f:
            return f.read()
    return None

def flash_device(uf2_path, mount_point, max_retries=3):
    """Flash the UF2 file to the device with retries."""
    for attempt in range(1, max_retries + 1):
        print(f"Flashing attempt {attempt}...")
        try:
            target_path = os.path.join(mount_point, os.path.basename(uf2_path))
            shutil.copy2(uf2_path, target_path)
            print("Copy successful, waiting for reboot...")
            # Wait for the device to unmount/reboot
            for _ in range(10):
                time.sleep(1)
                if not os.path.exists(mount_point):
                    print("Device rebooted successfully.")
                    return True
            print("Warning: Device still mounted after 10 seconds.")
        except Exception as e:
            print(f"Error during flashing: {e}")

        if attempt < max_retries:
            time.sleep(2)
    return False

def main():
    parser = argparse.ArgumentParser(description="RP2350 Device Auto-detection and Flashing Tool")
    parser.add_argument("uf2", help="Path to firmware.uf2")
    parser.add_argument("--retry", type=int, default=3, help="Max retries")
    args = parser.parse_args()

    if not os.path.exists(args.uf2):
        print(f"Error: {args.uf2} not found.")
        sys.exit(1)

    print("Searching for RP2350 device...")
    mount = find_rp2350_mount()
    if not mount:
        print("Device not found. Please ensure it's in BOOTSEL mode.")
        sys.exit(1)

    print(f"Found RP2350 at {mount}")
    metadata = read_metadata(mount)
    if metadata:
        print("Device Metadata:")
        print(metadata)

    if flash_device(args.uf2, mount, args.retry):
        print("Flashing complete.")
    else:
        print("Flashing failed.")
        sys.exit(1)

if __name__ == "__main__":
    main()
