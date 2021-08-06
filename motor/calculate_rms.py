import csv
import matplotlib.pyplot as plt
import numpy as np
import math

class RMS_calc(object):

    def __init__(self, filename, file_start):
        self.filename = filename
        self.file_start = file_start
        self.rms = []
        self.x = []
        self.y = [[],[],[]]

    def collect_data(self):
        with open("/home/pi/Documents/MOTOR_DATA_FOLDER/" + self.filename, 'r') as csvfile:
            plots = csv.reader(csvfile,delimiter=',')
            next(plots)
            for row in plots:
                self.x.append(int(row[0]))
                for i in range(4,7):
                    self.y[i-4].append(int(row[i]))
    #print(x)

    def calc(self): 
        for i in range(0, 3):
            temp_sum = 0
            temp_rms = 0
            for j in range(self.file_start,len(self.y[0])):
                temp_sum += (2 * ((self.y[i][j])**2) * (self.x[j] - self.x[j-1]))
            temp_rms = temp_sum/(self.x[len(self.y[0]) - 1] - self.x[self.file_start])
            self.rms.append(round((math.sqrt(temp_rms))/1000, 3))
        return self.rms

def main(filename_1, filename_2, file1_start, file2_start):
    rms1 = RMS_calc(filename_1, file1_start)
    rms2 = RMS_calc(filename_2, file2_start)

    rms1.collect_data()
    rms2.collect_data()

    rms1_val = rms1.calc()
    rms2_val = rms2.calc()

    return rms1_val, rms2_val
    

'''
if __name__ == "__main__":
    fileName = input("Enter filename:")
    collect_data()
    rms_calc()
    
    print(f"\nPhase A RMS: {rms[0]}")
    print(f"Phase B RMS: {rms[1]}")
    print(f"Phase C RMS: {rms[2]}")
'''
