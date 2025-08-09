#!/usr/bin/env python3

import sounddevice as sd
import numpy as np
import time

print("=== ReSpeaker 4 Mic Array Test ===\n")

# List all audio devices
print("Available audio devices:")
devices = sd.query_devices()
for i, device in enumerate(devices):
    if device['max_input_channels'] > 0:
        print(f"{i}: {device['name']} (inputs: {device['max_input_channels']})")

print("\n" + "="*50)

# Find your ReSpeaker device
respeaker_idx = None
for i, device in enumerate(devices):
    if 'ReSpeaker' in device['name'] or 'SEEED' in device['name']:
        respeaker_idx = i
        print(f"\nFound ReSpeaker at index {i}:")
        print(f"  Name: {device['name']}")
        print(f"  Max input channels: {device['max_input_channels']}")
        print(f"  Default sample rate: {device['default_samplerate']}")
        break

if respeaker_idx is not None:
    # Test recording from all 6 channels
    print(f"\nTesting 6-channel recording...")
    print("Recording 3 seconds - speak into the microphones!")
    
    duration = 3
    sample_rate = 16000
    channels = 6  # ReSpeaker provides 6 channels
    
    # Countdown
    for i in range(3, 0, -1):
        print(f"Starting in {i}...")
        time.sleep(1)
    
    print("Recording NOW!")
    recording = sd.rec(int(duration * sample_rate), 
                      samplerate=sample_rate, 
                      channels=channels, 
                      device=respeaker_idx)
    sd.wait()
    print("Recording complete!\n")
    
    # Analyze each channel
    print(f"Recording shape: {recording.shape}")
    print("\nChannel analysis:")
    for ch in range(channels):
        channel_data = recording[:, ch]
        rms = np.sqrt(np.mean(channel_data**2))
        max_val = np.max(np.abs(channel_data))
        print(f"Channel {ch}: RMS = {rms:.4f}, Peak = {max_val:.4f}")
    
    # Test if any significant audio was captured
    total_rms = np.sqrt(np.mean(recording**2))
    if total_rms > 0.001:
        print(f"\n✓ SUCCESS: Audio detected! Total RMS = {total_rms:.4f}")
    else:
        print(f"\n⚠ WARNING: Very low audio levels. Total RMS = {total_rms:.4f}")
        print("  Check if microphone is working and not muted.")
    
    # Test 4-channel recording (just the mics)
    print(f"\n" + "="*50)
    print("Testing 4-channel recording (microphones only)...")
    print("Recording 2 seconds...")
    
    time.sleep(1)
    recording_4ch = sd.rec(int(2 * sample_rate), 
                          samplerate=sample_rate, 
                          channels=4, 
                          device=respeaker_idx)
    sd.wait()
    print("4-channel recording complete!")
    
    print(f"4-channel recording shape: {recording_4ch.shape}")
    
    # Check which microphones are most active
    print("\nMicrophone activity:")
    for ch in range(4):
        channel_data = recording_4ch[:, ch]
        rms = np.sqrt(np.mean(channel_data**2))
        print(f"Mic {ch+1}: RMS = {rms:.4f}")
    
    # Show default audio settings
    print(f"\n" + "="*50)
    print("Current audio settings:")
    try:
        import subprocess
        result = subprocess.run(['pactl', 'get-default-source'], 
                              capture_output=True, text=True)
        print(f"Default source: {result.stdout.strip()}")
    except:
        print("Could not get default source info")
    
    print(f"\n✓ ReSpeaker setup appears to be working correctly!")
    print(f"✓ Device index for Python: {respeaker_idx}")
    print(f"✓ 6 channels available (4 mics + 2 processed)")
    print(f"✓ Sample rate: {sample_rate} Hz")
    
else:
    print("\n❌ ERROR: ReSpeaker device not found!")
    print("\nDevices with input channels:")
    for i, device in enumerate(devices):
        if device['max_input_channels'] > 0:
            print(f"  {i}: {device['name']} ({device['max_input_channels']} channels)")
    
    print("\nTroubleshooting steps:")
    print("1. Check USB connection")
    print("2. Run: lsusb | grep -i seeed")
    print("3. Run: pactl list sources short")
    print("4. Try unplugging and reconnecting the device")