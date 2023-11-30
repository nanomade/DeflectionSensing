
import serial
import threading
import csv
from PyQt5.QtWidgets import *
from PyQt5 import uic
import os

run = True

class GUI(QMainWindow):
    def __init__(self):
        super(GUI, self).__init__()

        uic.loadUi("Capture_GUI.ui", self)
        self.show()
        self.Start_capture_btn.clicked.connect(self.check_fields)
        self.Stop_capture_btn.clicked.connect(self.stop_capture)

    def check_fields(self):
        try:
            if "COM" in self.COM_name_input.text() and self.filename_input.text() != "":
                self.Start_capture_btn.setDisabled(True)
                self.Stop_capture_btn.setEnabled(True)

                filename = pick_name(self.filename_input.text())

                data_thread = threading.Thread(target=csv_setup, args=(self.COM_name_input.text(), filename))
                data_thread.start()

            else:
                message = QMessageBox()
                message.setText("Please enter a valid COM port and filename")
                message.exec_()
        except:
            message = QMessageBox()
            message.setText("Something went wrong :(")
            message.exec_()

    def stop_capture(self):
        global run
        run = False
        self.Start_capture_btn.setEnabled(True)
        self.Stop_capture_btn.setDisabled(True)

def serial_setup(ser):
    comma_count = 0
    dataLine = ser.readline().decode("utf-8")
    for i in dataLine:
        if i == ',':
            comma_count += 1

    return comma_count

def fetch_data(ser_obj, expected_length):
    dataLine = ser_obj.readline()
    #Arr = list(dataLine.decode("utf-8"))
    Arr = list(dataLine.decode("utf-8"))
    data = [0]*expected_length
    coef = 1
    j = expected_length-1

    for i in range(len(Arr)-2, 0, -1):
        if Arr[i] == ',':
            coef = 1
            j -= 1
        else:
            data[j] += int(Arr[i])*coef
            coef *= 10

    return data

def csv_setup(COM, filename):
    directory = os.getcwd()
    filename = directory+"\\"+filename

    ser = serial.Serial(port=COM, baudrate=38400)

    counted_commas = serial_setup(ser)
    packet_length = counted_commas + 1

    row_list = [[r"Time since last data package (ms)".encode(encoding="utf-8"),
                 r"Pressure (0.01 mbar)".encode(encoding="utf-8"),
                 r"Temperature (0.01 degC)".encode(encoding="utf-8"),
                 r"Reference Pressure (0.01 mbar)".encode(encoding="utf-8"),
                 r"Reference Temperature (0.01 degC)".encode(encoding="utf-8")]]

    with open(filename, "a+", encoding='utf-8', errors='ignore') as file:
        writer = csv.writer(file, quoting=csv.QUOTE_NONNUMERIC, delimiter=',')
        writer.writerows(row_list)
        while run:
            USBdata = fetch_data(ser, packet_length)
            writer.writerows([USBdata])
        file.close()

def pick_name(original_filename):
    available_filename = original_filename
    number = 1
    while (1):
        try:
            file = open(available_filename, "r", encoding='utf-8', errors='ignore')
            file.close()
            available_filename = original_filename + str(number)
            number += 1
        except FileNotFoundError:
            return available_filename

if __name__ == '__main__':
    try:
        app = QApplication([])
        window = GUI()
        app.exec()
        #input("Press any key to close")
    except:
        None








