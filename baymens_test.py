import time
# import board
# import adafruit_bh1750
import numpy as np
# from motorgo import BrakeMode, ControlMode, Plink
import math
# import adafruit_vl53l4cx
import matplotlib.pyplot as plt

'''
- add more sector points for improved discritization ~ 32 values
- implement odometry for movement update
- get histogram to work
- tune line following
'''


# PID Constants
KP_VEL = 0.015  # Proportional gain for velocity
KI_VEL = 0.001   # Integral gain for velocity
KD_VEL = 0.003   # Derivative gain for velocity

KP_ANGLE = 0.085  # Proportional gain for heading
KI_ANGLE = 0.00  # Integral gain for heading
KD_ANGLE = 0.0225  # Derivative gain for heading

BASE_PWR = 0.35  # Minimum power to move the motors


# i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# vl53 = adafruit_vl53l4cx.VL53L4CX(i2c)

print("VL53L4CX Simple Test.")

def bayesian_localization(map, sensor_reading, motion_step, belief, sensor_model, motion_model):
    """
    Performs a single iteration of the Bayesian localization update.

    :param map_data: List representing the binary map (1 = wall, 0 = empty space)
    :param sensor_reading: Measured distance from the sensor
    :param motion_step: Number of sectors moved (usually 1)
    :param belief: Current probability distribution over sectors
    :param sensor_model: Probability of correct vs. incorrect sensor readings
    :param motion_model: Probability distribution for movement uncertainty
    :return: Updated belief distribution
    """
    num_sectors = len(map)
    
    # 1. Motion Update (Prediction Step)
    new_belief = np.zeros(num_sectors)
    
    for i in range(num_sectors): # new sector index
        for j in range(num_sectors): # prev sector index
            # Probability of moving from sector j to i
            move_key = abs(i - (j + motion_step) % num_sectors)
            move_prob = motion_model.get(move_key, 0) 
            new_belief[i] += move_prob * belief[j]

    # 2. Sensor Update (Correction Step)
    for i in range(num_sectors):
        expected_reading = map[i]  # Expected sensor value
        match_prob = sensor_model[expected_reading][sensor_reading]  # Probability of this reading
        new_belief[i] *= match_prob  # Update probability

    # 3. Normalize the probability distribution
    new_belief /= np.sum(new_belief)

    # 4. Return updated probabilities
    return new_belief

def plot_belief_histogram(belief, iteration):
    plt.figure(figsize=(10, 5))
    plt.bar(range(len(belief)), belief, color='blue', alpha=0.7)
    plt.xlabel("Sector")
    plt.ylabel("Probability")
    plt.title(f"Probability Distribution of Localization - Iteration {iteration}")
    plt.ylim(0, 1)  # Keep the y-axis consistent for better visualization
    plt.xticks(range(len(belief)))
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    plt.show(block=True)

def main():
    # vl53.start_ranging()

    '''while True:
        while not vl53.data_ready:
            pass
        vl53.clear_interrupt()
        print("Distance: {} cm".format(vl53.distance))'''

    # Define map (1 = wall, 0 = empty space)
    map_data = [1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0]

    # Initial uniform belief
    belief = np.ones(len(map_data)) / len(map_data)

    # Define Sensor Model (P(sensor reading | true state))
    sensor_model = {
        0: {0: 0.8, 1: 0.2},  # 80% correct if empty, 20% false positive
        1: {0: 0.2, 1: 0.8}   # 80% correct if wall, 20% false negative
    }  # 90% accuracy

    # Define Motion Model (Probability of moving to correct sector)
    motion_model = {0: 0.8, 1: 0.1, 2: 0.1}  # 80% chance correct move, 10% overshoot/undershoot

    plot_belief_histogram(belief, 0)

    # Output most probable location
    iterations = 1000
    for i in range(iterations):
        readings = [np.random.choice([0, 1], p=[0.5, 0.5]) for _ in range(7)]  # Slightly biased towards walls
        sensor_reading = max(set(readings), key=readings.count)  # Majority vote

        motion_step = 1

        # Update belief
        belief = bayesian_localization(map_data, sensor_reading, motion_step, belief, sensor_model, motion_model)

        # Plot histogram for each iteration
        # plot_belief_histogram(belief, i + 1)

    print("New belief distribution: {}".format(belief))
    most_likely_sector = np.argmax(belief)
    print(f"Most likely sector: {most_likely_sector}, Probability: {belief[most_likely_sector]:.2f}")
    plot_belief_histogram(belief, "final")

if __name__ == "__main__":
    main()