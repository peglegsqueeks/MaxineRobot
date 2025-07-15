
import motoron
#from gpiozero import LED

REFERENCE_MV = 3300
PIN_ADDRESS = 21
TYPE = motoron.CurrentSenseType.MOTORON_24V16
MAX_ACCELERATION = 40
MAX_DECELERATION = 20
CURRENT_LIMIT= 13000


def configure_motor(mc, motor_id):
    # set acceleration / deceleration maximums
    mc.set_max_acceleration(motor_id, MAX_ACCELERATION)
    mc.set_max_deceleration(motor_id, MAX_DECELERATION)

    # set currnt limit on motor
    current_offset = mc.get_current_sense_offset(motor_id)
    limit = motoron.calculate_current_limit(CURRENT_LIMIT, TYPE, REFERENCE_MV, current_offset)
    mc.set_current_limit(motor_id, limit)

def initialise_motor():
    # initialise the motor controller
    mc = motoron.MotoronI2C()
    mc.reinitialize()
    mc.disable_crc()
    mc.clear_reset_flag()
    mc.disable_command_timeout()

    configure_motor(mc, 1)
    configure_motor(mc, 2)

    return mc


mc = initialise_motor()

# set speed to 0
mc.set_speed(1, 0)
mc.set_speed(2, 0)


# switch brake ON (pin off)
brake_line = LED(PIN_ADDRESS)
brake_line.off()
        