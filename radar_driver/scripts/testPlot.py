import matplotlib.pyplot as plt

ranges = []
velRad = []

with open("test7.txt", "r") as f:
    for line in f:
        if "f_Range " in line:
            lines = line.split(" ")
            ranges.append(float(lines[1]))
        if "f_VrelRad " in line:
            lines = line.split(" ")
            velRad.append(float(lines[1]))

fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)

ax1.plot(ranges[50:160], color='r')
ax1.set_title("Ranges")
ax2.plot(velRad[13500:15000], color='b')
ax2.set_title("VelRad")
plt.show()

