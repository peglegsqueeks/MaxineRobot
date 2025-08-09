import speech_recognition as sr

# Get all microphone names with their indices
mics = sr.Microphone.list_microphone_names()

print("=== Speech Recognition Microphone Devices ===")
print("Index : Device Name")
print("-" * 60)

respeaker_index = None

for index, name in enumerate(mics):
    print(f"{index:2d}    : {name}")
    
    # Look for ReSpeaker device
    if 'ReSpeaker' in name or 'SEEED' in name:
        respeaker_index = index
        print(f"       ^^^ FOUND RESPEAKER AT INDEX {index} ^^^")

print("-" * 60)

if respeaker_index is not None:
    print(f"\n✓ ReSpeaker found at index: {respeaker_index}")
    print(f"✓ Use device_index={respeaker_index} in your speech recognition code")
    
    # Test the microphone
    print(f"\nTesting microphone at index {respeaker_index}...")
    try:
        r = sr.Recognizer()
        mic = sr.Microphone(device_index=respeaker_index)
        
        with mic as source:
            print("Adjusting for ambient noise... (this may take 2 seconds)")
            r.adjust_for_ambient_noise(source)
            print("Microphone test successful!")
            
    except Exception as e:
        print(f"Error testing microphone: {e}")
        
else:
    print("\n❌ ReSpeaker not found in microphone list")
    print("Available devices with 'USB' or 'Audio' in the name:")
    for index, name in enumerate(mics):
        if 'USB' in name or 'Audio' in name:
            print(f"  {index}: {name}")