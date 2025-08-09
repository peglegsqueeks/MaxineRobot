import speech_recognition as sr
import time
import os
os.environ['ALSA_DISABLE_CONF'] = '1'

stop_it = False

r = sr.Recognizer()
mic = sr.Microphone(device_index=24)
# Words that sphinx should listen closely for. 0-1 is the sensitivity
# of the wake word.
keywords = [("red robot", 1), ("initiate search mode", 1), ]
source = mic

def callback(recognizer, audio):  # this is called from the background thread
    global stop_it
    try:
        speech_as_text = r.recognize_sphinx(audio, keyword_entries=keywords)
        print(speech_as_text)
        # Look for your "Ok Google" keyword in speech_as_text
        if "robot" in speech_as_text or "red" in speech_as_text:
            stop_it=True
            recognize_main()

    except sr.UnknownValueError:
        print("Oops! Didn't catch that")

def recognize_main():
    print("Recognizing Main...")
    print('Hey I am Listening ')
    audio = r.listen(source)
    # interpret the user's words however you normally interpret them
    a2t=r.recognize_sphinx(audio)
    print(a2t)
    print('found meaningful data')

just_try_and_stop_me = r.listen_in_background(source, callback)

while True:
    if stop_it:
        just_try_and_stop_me(wait_for_stop=True)
        break
    time.sleep(0.1)