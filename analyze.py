import numpy as np
import matplotlib.pyplot as plt

backLegSensorValues = np.load("data/backLegSensorValues.npy")
frontLegSensorValues = np.load("data/frontLegSensorValues.npy")

print(backLegSensorValues)

plt.plot(backLegSensorValues, label="Back Leg", linewidth=3)
plt.plot(frontLegSensorValues, label="Front Leg")
plt.legend()
plt.show()
