#!/usr/bin/env python3
"""
Find the correct ReSpeaker INPUT device (not output)
"""

import sounddevice as sd

def find_respeaker_input():
    print("=== Finding ReSpeaker INPUT Device ===")
    
    devices = sd.query_devices()
    respeaker_devices = []
    
    for i, device in enumerate(devices):
        if 'ReSpeaker' in device['name'] or 'SEEED' in device['name']:
            print(f"\nDevice {i}: {device['name']}")
            print(f"  Input channels: {device['max_input_channels']}")
            print(f"  Output channels: {device['max_output_channels']}")
            print(f"  Sample rate: {device['default_samplerate']}")
            
            if device['max_input_channels'] > 0:
                print(f"  ✅ THIS IS AN INPUT DEVICE")
                respeaker_devices.append(i)
            else:
                print(f"  ❌ Output only (no microphones)")
    
    if respeaker_devices:
        correct_device = respeaker_devices[0]
        print(f"\n🎯 USE DEVICE INDEX: {correct_device}")
        return correct_device
    else:
        print("\n❌ No ReSpeaker input devices found")
        print("\nAll devices with inputs:")
        for i, device in enumerate(devices):
            if device['max_input_channels'] > 0:
                print(f"  {i}: {device['name']} ({device['max_input_channels']} inputs)")
        return None

if __name__ == "__main__":
    device_idx = find_respeaker_input()
    
    if device_idx is not None:
        print(f"\nTo fix your speech recognition:")
        print(f"Change device_index to {device_idx} in your script")
    else:
        print("\nTroubleshooting:")
        print("1. Check USB connection")
        print("2. Run: lsusb | grep -i seeed")
        print("3. Try unplugging and reconnecting ReSpeaker")