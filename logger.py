import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv


#Feito segundo as instruções disponíveis em: https://www.youtube.com/watch?v=PhDPnjF3_tA
#Créditos para: The Bored Robot



SERIAL_PORT = "COM8"
BAUD_RATE = 115200


ser = serial.Serial(SERIAL_PORT, BAUD_RATE)


t_vals =[]
temp_vals = []
temp_ref = []

def read_process_data():
    line = ser.readline().decode('utf-8').strip()
    sensorValues = line.split(", ")

    t_vals.append(float(sensorValues[0]))
    temp_vals.append(float(sensorValues[1]))
    temp_ref.append(37)

def update_plot(frame):
    read_process_data()
    plt.cla()
    plt.plot(t_vals, temp_vals, label="Temperatura (C)")
    plt.plot(t_vals,temp_ref , label="Referência (C)")
    plt.xlabel('Tempo (s)')
    plt.ylabel('Temperatura (C)')
    plt.grid(True)
    plt.legend()


def on_close(event):
    with open('tmp_data_csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['tempo', 'temperatura'])
        for t, s in zip(t_vals, temp_vals):
            writer.writerow([t, s])




fig, ax = plt.subplots()
fig.canvas.mpl_connect('close_event', on_close)


ani = FuncAnimation(fig, update_plot, interval=10)
plt.show()