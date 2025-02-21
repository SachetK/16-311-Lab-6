import time
import board
import adafruit_bh1750
import numpy as np
from motorgo import BrakeMode, ControlMode, Plink
import math
import adafruit_vl53l4cx 

# PID Constants
KP_VEL = 0.015  # Proportional gain for velocity
KI_VEL = 0.001   # Integral gain for velocity
KD_VEL = 0.003   # Derivative gain for velocity

KP_ANGLE = 0.085  # Proportional gain for heading
KI_ANGLE = 0.00  # Integral gain for heading
KD_ANGLE = 0.0225  # Derivative gain for heading

BASE_PWR = 0.35  # Minimum power to move the motors

# Kalman Filter Parameters
Q_angle = 0.02   # Process noise variance for the angle
Q_bias = 0.005   # Process noise variance for the bias
R_measure = 0.01  # Measurement noise variance

class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.angle = -np.pi/2.0 # np.pi /4.0  # Estimated angle
        self.bias = 0.0   # Estimated bias
        self.P = np.array([[1, 0], [0, 1]])  # Error covariance matrix
        self.prev_error_vel = 0.0
        self.prev_error_angle = 0.0
        self.integral_vel = 0.0
        self.integral_angle = 0.0
        self.offset_z = 0.0

        self.distance 
        self.goal_sector = 0
        self.sector = None
        self.map = {1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}
        self.distribution = np.ones(16)/16

    def update_kalman(self, new_angle, new_rate, dt):
        # Predict step
        rate = new_rate - self.bias
        self.angle += rate * dt
        
        # Update estimation error covariance
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += Q_bias * dt
        
        # Compute Kalman gain
        S = self.P[0][0] + R_measure
        K = [self.P[0][0] / (S * 2), self.P[1][0] / (S * 2)]
        
        # Update step
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y
        
        # Update error covariance matrix
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp
        
        return self.angle
    
    def odometry(self, v, w, t):
        k00 = v * np.cos(self.angle)
        k01 = v * np.sin(self.angle)
        k02 = w 

        k10 = v * np.cos(self.angle + ((t/2) * k02))
        k11 = v * np.sin(self.angle + ((t/2) * k02))
        k12 = w

        k20 = v * np.cos(self.angle + ((t/2) * k12))
        k21 = v * np.sin(self.angle + ((t/2) * k12))
        k22 = w

        k30 = v * np.cos(self.angle + ((t/2) * k22))
        k31 = v * np.sin(self.angle + ((t/2) * k22))
        k32 = w

        self.x += (t / 6) * (k00 + 2 * (k10 + k20) + k30)
        self.y += (t / 6) * (k01 + 2 * (k11 + k21) + k31)
        dtheta = (t / 6) * (k02 + 2 * (k12 + k22) + k32)

        return self.x, self.y, dtheta
    
    def pid_control(self, target_x, target_y, error_vel, dt):
        # Compute distance and heading error
        # error_dist = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
        target_angle = math.atan2(target_y - self.y, target_x - self.x)
        error_angle = (target_angle - self.angle + np.pi) % (2 * np.pi) - np.pi

        #print(f'error angle = {error_angle * 180 / np.pi} '
              #f'target angle = {target_angle * 180 / np.pi}')
    
        # PID for velocity
        self.integral_vel += error_vel * dt
        derivative_vel = (error_vel - self.prev_error_vel) / dt
        self.prev_error_vel = error_vel
        vel_command = KP_VEL * error_vel + KI_VEL * self.integral_vel + KD_VEL * derivative_vel

        # PID for heading
        self.integral_angle += error_angle * dt
        derivative_angle = (error_angle - self.prev_error_angle) / dt
        self.prev_error_angle = error_angle
        angle_command = KP_ANGLE * error_angle + KI_ANGLE * self.integral_angle + KD_ANGLE * derivative_angle

        return angle_command, vel_command
    
    def update_self(self, imu, v, w, dt):
        # Update robot position and angle using odometry
        x, y, dtheta = self.odometry(v, w, dt)

        # Get IMU gyro data (Z-axis angular velocity)
        omega_z = imu.gyro[2] - self.offset_z 

        # Compute two different angles:s
        # Runge-Kutta computed angle
        odometry_angle = self.angle + dtheta   
        # Gyro-integrated angle
        gyro_angle = self.angle + omega_z * dt 

        # Weighted fusion: favor odometry (80%) over gyro (20%)
        self.angle = 0.9 * odometry_angle + 0.1 * gyro_angle

        # Apply Kalman filter using the fused angle as the measurement
        # refined_angle = self.update_kalman(fused_angle, omega_z, dt)

        return x, y, self.angle
    
    #   Currently assuming called in discrete intervals. 
    #   ****needs to be modified to be tolerant to measuement errors**** 
    #   consider averaging distance readings over a range and using that as 
    #   input to mimic discretized symbol. 
    #   Perhaps it is required to add an intermediary distance. 
    def update_probabalistic(self, distance):
        # Likelihood: P(distance | map)
        likelihood = np.array([0.9 if d == distance else 0.1 for d in self.map])

        # Apply Bayes' rule: P(map | distance) ∝ P(distance | map) * P(map)
        self.distribution *= likelihood

        # normalize to sum to 1
        self.distribution /= np.sum(self.distribution)

        # set maximum
        self.sector = max(self.distribution)
        return self.sector


def calibrate_gyro(imu,samples=100):
    print("Calibrating gyroscope... Please keep the sensor still.")
    
    time.sleep(0.05)
    sum_z = 0
    for _ in range(samples):
        gz = imu.gyro[2]
        sum_z += gz
        time.sleep(0.01)  # Small delay between samples
    
    offset_z = sum_z / samples
    
    print(f"Calibration complete! Offset: Z={offset_z}")
    return offset_z

def load_waypoints(filename):
    waypoints = []
    with open(filename, 'r') as file:
        for line in file:
            x, y = map(float, line.strip().split(','))
            waypoints.append((x, y))
    return waypoints

def main():
    plink = Plink()
    imu = plink.imu

    i2c = board.I2C()
    vl53 = adafruit_vl53l4cx.VL53L4CX(i2c)
    sensor = adafruit_bh1750.BH1750(i2c)

    left_motor = plink.channel3
    right_motor = plink.channel1

    print("Connecting...")
    plink.connect()
    print("Connected.")
  
    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    robot = Robot()

    prev_time = time.time()
    start_time = prev_time

    # Robot Parameters (in)
    wheel_base = 4.375
    wheel_radius = 0.85

    heading_threshold = 0.005
    probability_threshold = 0.99

    #robot.offset_z = calibrate_gyro(imu)

              
    while robot.distribution(robot.goal_sector) < probability_threshold:
        while not v153.data_ready:
            pass 
        v153.clear_interrupt()
        print("Distance: {} cm", format())
        distance = v153.distance
        lux = sensor.lux
        print("%.2f Lux" % lux)

        robot.update_probabalistic(distance)

        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time

        left_velocity = -left_motor.velocity * wheel_radius
        right_velocity = right_motor.velocity * wheel_radius

        v = (left_velocity + right_velocity) / 2
        w = (right_velocity - left_velocity) / wheel_base


        error = abs(left_velocity) - abs(right_velocity)  # Speed difference

        v153.start_ranging() 

        x,y,theta = robot.update_self(imu, v, w, dt)

        angle_command, correction = robot.pid_control(lux, error, dt)
        # correction > 0 when left wheel faster

        # angle_command is negative, heading facing left of the waypoint, 
        # needs to turn right
        if ((angle_command) < -heading_threshold):
            left_pwr = max(min(-BASE_PWR + angle_command + correction,1.0),-1.0) 
            right_pwr = max(min(BASE_PWR - angle_command + correction,1.0),-1.0) 
            print("TURNING RIGHT")
        #needs to turn left
        elif ((angle_command) > heading_threshold): 
            left_pwr = max(min(BASE_PWR + angle_command - correction,1.0),-1.0)
            right_pwr = max(min(-BASE_PWR - angle_command - correction,1.0),-1.0)
            print("TURNING LEFT")
        else: 
            left_pwr = -BASE_PWR - 0.1 + correction
            right_pwr = -BASE_PWR - 0.1 - correction
            print("STRAIGHT")

        left_motor.power_command = left_pwr
        right_motor.power_command = right_pwr

        #print(f'left_pwr = {-left_pwr:.3f}, right_pwr = {-right_pwr:.3f}')
        #print(f'x = {x:.3f}, y = {y:.3f}, θ = {(theta * 180 / np.pi) % 360:.2f}°, dt = {dt:.3f}s')

        time.sleep(0.01)

    
    left_motor.power_command = 0
    right_motor.power_command = 0
    plink.reset()

if __name__ == "__main__":
    main()
