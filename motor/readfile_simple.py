import csv
import matplotlib.pyplot as plt
import numpy as np

x = []
y = [[],[],[],[],[]]
fileName = ''


def collect_data():
    with open('/home/pi/Documents/MOTOR_DATA_FOLDER/' + fileName, 'r') as csvfile:
        plots = csv.reader(csvfile,delimiter=',')
        next(plots)
        for row in plots:
            x.append(int(row[0]))
            for i in range(1,5):
                y[i-1].append(float(row[i]))

def graph_data():
    fig, axs = plt.subplots(2)
    axs[1].set_ylim(0, 35)
    axs[0].plot(x, y[0])
    for i in range(1, 4):
        axs[1].plot(x, y[i])
    plt.xlabel('time [us]')
    axs[1].legend(["Phase A RMS", "Phase B RMS", "Phase C RMS"])
    axs[0].legend(["RPM"])
    plt.show()

if __name__ == "__main__":
    fileName = input("Enter filename:")
    collect_data()
    graph_data()
    #collect_rpm()
    
