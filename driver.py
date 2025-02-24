import time
import board
import adafruit_bh1750
import numpy as np
from motorgo import BrakeMode, ControlMode, Plink

'''
PART 1: LINE FOLLOWING 
'''

def clamp(val, min_val, max_val):
    if val < min_val:
        return min_val
    elif val > max_val:
        return max_val
    else:
        return val

def main():
    #board.reset()
    # Create a Plink object, the main interface to the MotorGo board
    plink = Plink()
    # The first thing to set up for a Plink is the power supply voltage.
    # This is the voltage you are providing to the Plink on the barrel jack.
    # If this is the battery, make sure this is the charged voltage.
    #plink.power_supply_voltage = 9.0

    # The Plink object has 4 MotorChannel objects, corresponding to the 4 motor channels
    # on the board
    # You can access them directly: plink.channel1
    # Or you can save references as local variables for convenience:
    left_motor = plink.channel1
    right_motor = plink.channel3
 
    # Next, you need to set the motor voltage limit.
    # This is the maximum voltage your motor channels will output to protect the motors.
    # The voltage limit is 0 by default, which means the motors will not move if this is not set.
    left_motor.motor_voltage_limit = 6.0
    right_motor.motor_voltage_limit = 6.0
    # plink.channel3.motor_voltage_limit = 6.0
    # plink.channel4.motor_voltage_limit = 6.0
    
    # Finally, connect to the MotorGo board and psuh the configuration
    print("connecting")
    plink.connect()
    print("connected")

    # You can configure how you want to control the motor channels.
    # Power mode: Set the power of the motor in the range [-1, 1]
    #            This directly corresponds to setting a voltage to the motor
    #
    # Velocity mode: Set the velocity of the motor in rad/s
    #              This mode requires setting the velocity PID gains
    #              It also requires an encoder to be connected to the motor
    
    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    # right_motor.control_mode = ControlMode.VELOCITY
    #plink.channel3.control_mode = ControlMode.POWER
	#plink.channel4.control_mode = ControlMode.POWER
 
    # If you are using ControlMode.VELOCITY, you must set the velocity PID gains
    # right_motor.set_velocity_pid_gains(-1, -0.1, 0) #orig 4, 1, 0
    # left_motor.set_velocity_pid_gains(1, 0.1, 0) 
    
    i2c = board.I2C()  # uses board.SCL and board.SDA
    #i2c  = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
    sensor = adafruit_bh1750.BH1750(i2c) 
   
    target = 40.0

    base_speed = 0.4 # (rad/s)
    
    integral = 0
    derivative = 0
    previous_error = 0
    
    error = 0.0
    
    # Kp = 0.08
    Kp = 0.1
    Ki = 0.01
    Kd = 0.5

    try:
        while True:
            # Light Sensor PID Logic 
            current_lux = sensor.lux
            print(f"lux_read {current_lux}")
            
            error = current_lux - target
            
            #integral += error
            #derivative = error - previous_error
            correction = Kp * error #+ Kd * derivative#+ Ki * integral + Kd * derivative

            #update previous error 
            print(f'error is {error}, correction is {correction}')
            previous_error = error
            
            power = clamp(base_speed + correction, base_speed, 1.0) if correction > 0 else clamp(-base_speed-correction, -1.0, -base_speed)


            if abs(error) > 3.5: #4.5
                if correction > 0:
                    right_motor.power_command = clamp(base_speed + correction, base_speed, 1.0)
                    left_motor.power_command = clamp(-base_speed - correction, -1.0, -base_speed)
                    print(" move left")
                elif correction < 0:
                    right_motor.power_command = clamp(-base_speed - correction, -1.0, -base_speed)
                    left_motor.power_command = clamp(base_speed + correction, base_speed, 1.0)
                    print("move right")
            else:
                left_motor.power_command = -base_speed
                right_motor.power_command = -base_speed
    
            
            time.sleep(0.05)
            
            
            # if  (lux>=30 and lux <= 45):  # Darker surface → Move right
            #     print("straight")
            #     left_motor.velocity_command = 4 
            #     # Right wheel moves faster, so robot turns right
            #     right_motor.velocity_command = -4 
            #     #time.sleep(0.1)

            # elif (lux<=70 and lux>45):  # White line → Move left
            #      # Left wheel moves faster, so robot turns left
            #     print("left")
            #     left_motor.velocity_command = 1 
            #     right_motor.velocity_command = -4
            # time.sleep(0.01)
    except KeyboardInterrupt:
        print("program terminated")
        


if __name__ == "__main__":

    main()

