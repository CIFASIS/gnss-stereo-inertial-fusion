import numpy as np
import matplotlib.pyplot as plt

input_data = "timestamps.txt"
data = np.loadtxt(input_data, delimiter=",")
#print(data[:,0].shape)

plt.scatter(range(data.shape[0]), data[:,1] - data[:,0], s=1)
plt.show()
#print(data)


