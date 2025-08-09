#!/usr/bin/env python3
"""
Exact copy of what worked in your original test - 6 channels, device 24
"""

import json
import queue
import sounddevice as sd
import vosk
import time
import threading
import os

# Suppress ALSA warnings
os.environ['ALSA_DISABLE_CONF'] = '1'

class WorkingVoskRecognition:
    def __init__(self, model_path="vosk-model-small-en-us-0.15"):
        # Use EXACT same settings as your working test
        self.device_index = 24  # From your successful test
        self.sample_rate = 16000
        self.channels = 6  # ReSpeaker native mode that worked
        
        print(f"Using device index: {self.device_index}")
        print(f"Using {self.channels} channels (processing channel 0)")
        
        # Load Vosk model
        if not os.path.exists(model_path):
            print(f"❌ Model not found at {model_path}")
            exit(1)
            
        print(f"Loading Vosk model...")
        vosk.SetLogLevel(-1)
        self.model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(self.model, 16000)
        print("✅ Vosk model loaded!")
        
        # Simple wake words
        self.wake_words = ["right rotate", "maxine", "computer"]
        
        self.listening_for_command = False
        self.stop_listening = False
        self.audio_queue = queue.Queue()
        self.recognition_thread = None
        
    def audio_callback(self, indata, frames, time, status):
        """Use channel 0 - same as your working test"""
        if status:
            print(f"Audio status: {status}")
        
        # Extract channel 0 (the one with RMS=0.2043 in your test)
        channel_0 = indata[:, 0]
        
        # Convert to bytes for Vosk
        audio_data = (channel_0 * 32767).astype('int16').tobytes()
        
        try:
            self.audio_queue.put_nowait(audio_data)
        except queue.Full:
            pass
    
    def recognition_worker(self):
        """Simple recognition worker"""
        print("🎤 Recognition started")
        
        while not self.stop_listening:
            try:
                audio_data = self.audio_queue.get(timeout=0.1)
                
                if self.rec.AcceptWaveform(audio_data):
                    result = json.loads(self.rec.Result())
                    text = result.get('text', '').strip()
                    
                    if text:
                        print(f"Heard: '{text}'")
                        
                        if self.listening_for_command:
                            self.process_command(text)
                            self.listening_for_command = False
                        else:
                            self.check_wake_words(text)
                                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error: {e}")
                
        print("🛑 Recognition stopped")
    
    def check_wake_words(self, text):
        """Check wake words"""
        for wake_word in self.wake_words:
            if wake_word in text.lower():
                print(f"🎯 Wake word: '{wake_word}'")
                print("🎤 Listening for command...")
                self.listening_for_command = True
                return True
        return False
    
    def process_command(self, command):
        """Process command"""
        print(f"✅ Command: '{command}'")
        
        if "lidar" in command.lower():
            print("🔍 LIDAR activated")
        elif "camera" in command.lower():
            print("📷 Camera activated") 
        elif "stop" in command.lower():
            print("🛑 Stopped")
        else:
            print("❓ Unknown command")
            
        print("🔊 Listening for wake words...")
        
    def start(self):
        """Start system"""
        print("=== Working Vosk Recognition ===")
        print(f"Device: {self.device_index}")
        print(f"Channels: {self.channels}")
        print(f"Wake words: {self.wake_words}")
        print("=" * 40)
        
        self.recognition_thread = threading.Thread(target=self.recognition_worker, daemon=True)
        self.recognition_thread.start()
        
        try:
            # Use EXACT same parameters as your working test
            with sd.InputStream(
                samplerate=self.sample_rate,
                device=self.device_index,
                channels=self.channels,
                callback=self.audio_callback,
                blocksize=1600
            ):
                print("🔊 Listening... try 'right rotate'")
                
                while not self.stop_listening:
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\n🛑 Shutting down...")
        except Exception as e:
            print(f"❌ Error: {e}")
            # Show device info for debugging
            print("\nDevice info:")
            devices = sd.query_devices()
            if self.device_index < len(devices):
                print(f"Device {self.device_index}: {devices[self.device_index]}")
        finally:
            self.stop_listening = True

def main():
    recognizer = WorkingVoskRecognition()
    recognizer.start()

if __name__ == "__main__":
    main()