import data
import matplotlib.pyplot as plt
trainset = data.JointLandmarkDataset("data/ver4_1/joints.pt", "data/ver4_1/landmarks.pt")
valset = data.JointLandmarkDataset("data/ver4_val/joints.pt", "data/ver4_val/landmarks.pt", phase=3.14/12)

for act in range(valset.landmark.shape[0]):
    plt.plot(valset.landmark[act,:,1], valset.landmark[act,:,2])
plt.show()