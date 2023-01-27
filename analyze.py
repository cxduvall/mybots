import numpy as np
import matplotlib.pyplot as plt

backLegSensorValues = np.load("data/backLegSensorValues.npy")
frontLegSensorValues = np.load("data/frontLegSensorValues.npy")

backTargetAngles = np.load("data/backTargetAngles.npy")
frontTargetAngles = np.load("data/frontTargetAngles.npy")

#print(backLegSensorValues)

plt.plot(backTargetAngles, label="Back Target Angles")
plt.plot(frontTargetAngles, label="Front Target Angles")
plt.plot(backLegSensorValues, label="Back Leg", linewidth=3)
plt.plot(frontLegSensorValues, label="Front Leg")
plt.legend()
plt.show()
