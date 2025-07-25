import Jetson.GPIO as GPIO
import time
import motoron
# Pin Definitions
output_pin = 7  # Jetson Board Pin 7
mc = motoron.MotoronI2C(bus=7)
mc.reinitialize()  # Bytes: 0x96 0x74
mc.disable_crc()   # Bytes: 0x8B 0x04 0x7B 0x43
# Clear the reset flag, which is set after the controller reinitializes and
# counts as an error.
mc.clear_reset_flag()  # Bytes: 0xA9 0x00 0x04
mc.disable_command_timeout()
mc.set_max_acceleration(1, 200)
mc.set_max_deceleration(1, 10)
mc.set_max_acceleration(2, 200)
mc.set_max_deceleration(2, 10)

def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # Jetson board numbering scheme
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)

    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.HIGH
    GPIO.output(output_pin, curr_value)
    print("Outputting {} to pin {}".format(curr_value, output_pin))
    
    try:
        while True:
            time.sleep(2)
            mc.set_speed(1, 300)
            mc.set_speed(2, 300)
            time.sleep(3)
            mc.set_speed(1,0)
            mc.set_speed(2,0)
            time.sleep(1)
            mc.set_speed(1, -300)
            mc.set_speed(2, -300)
            time.sleep(3)
            mc.set_speed(1,0)
            mc.set_speed(2,0)
            print("Outputting {} to pin {}".format(curr_value, output_pin))
            time.sleep(1)
    finally:
        mc.set_speed(1, 0)
        mc.set_speed(2, 0)
        time.sleep(1)
        GPIO.output(output_pin, 0)
        GPIO.cleanup()

if __name__ == '__main__':
    main()
