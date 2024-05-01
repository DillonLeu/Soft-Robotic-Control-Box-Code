# Import modules
from tkinter import *
import customtkinter
from itertools import (permutations, count)
import threading
from threading import Timer
import RPi.GPIO as GPIO
import time
from time import sleep
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure 
import random
import smbus
import numpy as np
import datetime
import os
import csv
import board
import busio
import adafruit_mcp4725
# Set the appearance mode and default color theme for the GUI
customtkinter.set_appearance_mode("dark")
customtkinter.set_default_color_theme("dark-blue")
# Initialize the tkinter root window
root = customtkinter.CTk()
root.title('Soft Robot GUI')
root.attributes("-fullscreen", True)
# Get screen dimensions
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
root.geometry(f"{screen_width}x{screen_height}+0+0")
# Print screen dimensions
print("Screen width:", screen_width)
print("Screen height:", screen_height)
# initialise global variable
main_frame = {}
pickValve_status = {}
manualValveControl = {}
frequencyValve = {}
frequencyInputValve = {}
frequencySliderValve = {}
dutyCycleInputValve = {}
dutyCycleSliderValve = {}
decreaseDutyCycleValve = {}
increaseDutyCycleValve = {}
decreaseFrequencyValve = {}
increaseFrequencyValve = {}
manualValveControl = {}
pwm = {}
currentManualValveControlState = {}
pwmStarted = {}
pwmStarted[1] = False
pwmStarted[2] = False
pwmStarted[3] = False
pwmStarted[4] = False
startMotor = False
startPressureRegulator = False
saveData = False
currentValves_status = 4 * ["off"]
sequenceOrderOptions = []
valveTurnOn = []
differentValveFrequencies = []
differentValveDutyCycle = []
increasePressure = None
decreasePressure = None
increaseSetMotorSpeed = None
decreaseSetMotorSpeed = None
increaseSimilarFrequency = None
decreaseSimilarFrequency = None
increaseSimilarDutyCycle = None
decreaseSimilarDutyCycle = None
decreaseTimeInterval = None
increaseTimeInterval = None
timeIntervalInput = None
pressure_slider = None
micropump_slider = None
frequency_slider = None
dutycycle_slider = None
timeIntervalSlider = None
setPressure = None
pressureInput = None
pressureUnit = None
setMotorSpeed = None
motorSpeedInput = None
motorSpeedUnit = None
similarFrequencyLabel = None
similarFrequencyInput = None
similarFrequencyUnit = None
similarDutyCycleLabel = None
similarDutyCycleInput = None
similarDutyCycleUnit = None
manualControlLabel = None
autoControlLabel = None
valve1Label = None
valve2Label = None
valve3Label = None
valve4Label = None
frequency1Label = None
frequency2Label = None
frequency3Label = None
frequency4Label = None
frequency1Unit = None
frequency2Unit = None
frequency3Unit = None
frequency4Unit = None
dutyCycleValve1Label = None
dutyCycleValve2Label = None
dutyCycleValve3Label = None
dutyCycleValve4Label = None
dutyCycleValve1Unit = None
dutyCycleValve2Unit = None
dutyCycleValve3Unit = None
dutyCycleValve4Unit = None
sequencingOrderLabel = None
selectSequenceOrderInput = None
timeIntervalLabel = None
timeIntervalInput = None
timeIntervalUnit = None
currentButton_page = None
dutyCycleLiveDataSwitch = None
runTimeDurationInput = None
dcLiveDataWindow = None
pressureLiveDataWindow = None
currentSetPressure = 0.0
currentSetMotorSpeed = 0
currentSetFrequency = 0.0
currentSetFrequency1 = 0.0
currentSetFrequency2 = 0.0
currentSetFrequency3 = 0.0
currentSetFrequency4 = 0.0
currentSetDutyCycle = 0
currentSetDutyCycle1 = 0
currentSetDutyCycle2 = 0
currentSetDutyCycle3 = 0
currentSetDutyCycle4 = 0
currentSetSequence1 = 0 
currentSetSequence2 = 0 
currentSetSequence3 = 0 
currentSetSequence4 = 0 
currentSetTimeInterval = 0
currentSetRunTimeDuration = 0 
currentManualValveControlState[1] = 0
currentManualValveControlState[2] = 0
currentManualValveControlState[3] = 0
currentManualValveControlState[4] = 0
selected_sequence = ""   
lightDarkMode = customtkinter.StringVar(value = "dark")
pin_val = [17, 27, 22, 5]
motorSpeedPin = [23, 25]
motorSpeedPWM = None
MULTIPLEXER_ADDRESS = 0x70
PRESSURE_SENSOR_ADDRESS = 0x28
pressureReader = []



# Hide the indicator
def hide_indicator():
    global lightDarkMode
    # Check the mode and update the foreground color of UI elements accordingly
    if lightDarkMode.get() == "dark":
        pressure_menu.configure(fg_color = "#2F2F2F")
        controlMode.configure(fg_color = "#2F2F2F")
        valveSequencing.configure(fg_color = "#2F2F2F")
    elif lightDarkMode.get() == "light":
        pressure_menu.configure(fg_color = "#F0F4C3")
        controlMode.configure(fg_color = "#F0F4C3")
        valveSequencing.configure(fg_color = "#F0F4C3")
# show indicator when page selected
def indicate(lb, page):
    global lightDarkMode
    global currentButton_page
    hide_indicator()
    delete_pages()
    # Change the foreground color of the selected label based on the mode
    if lightDarkMode.get() == "dark":
        lb.configure(fg_color = "#4169E1")
    elif lightDarkMode.get() == "light":
        lb.configure(fg_color = "#A5D6A7")
    
    # Set the currentButton_page to the selected label
    currentButton_page = lb
    page()

# Delete existing pages
def delete_pages():
    for frame in main_frame.values():
        frame.destroy()

# ****************** hardware programming ******************************************
# Selects the channels on the multiplexer.
def selectChannels(channels):
    global MULTIPLEXER_ADDRESS
    try:
        multiplexer_fd = smbus.SMBus(1)
        multiplexer_fd.write_byte(MULTIPLEXER_ADDRESS, channels)
        time.sleep(0.000001)
        multiplexer_fd.close()
    except IOError:
        print("Unable to open I2C device for the multiplexer")
        return

# Reads pressure values from pressure sensors.
def readPressure():
     global PRESSURE_SENSOR_ADDRESS
     sensor_fd = smbus.SMBus(1)
     pressurePsi = []
     for channel in range(2, 4):
         selectChannels(1 << channel)
         pressure_value_raw = sensor_fd.read_i2c_block_data(PRESSURE_SENSOR_ADDRESS, 0, 2)
         pressure_value = (pressure_value_raw[0] << 8) | pressure_value_raw[1]
         p_max = 15.0
         p_min = 0.0
         percentage_Output = pressure_value/16383
         pressure_Applied = round(((percentage_Output - 0.1) * (p_max - p_min))/0.8, 2) 
         pressurePsi.append(pressure_Applied)
     sensor_fd.close()
     return pressurePsi

# Continuously reads pressure values from pressure sensors and updates the pressureReader variable.     
def sensor_feedback():
    global pressureReader
    while True:
        pressureReader = readPressure()

def gpio_initialisation():
    global pin_val
    global motorSpeedPin
    # Set all GPIO pins to LOW
    for pin_index in range(len(pin_val)):
        GPIO.output(pin_val[pin_index], GPIO.LOW)
    GPIO.output(motorSpeedPin[1], GPIO.LOW)

def motorSpeedAdjustment():
    global startMotor
    global motorSpeedPin
    global currentSetMotorSpeed
    global motorSpeedPWM
    # Start the motor and set the PWM
    startMotor = True
    print(currentSetMotorSpeed)
    GPIO.output(motorSpeedPin[1], GPIO.HIGH)
    motorSpeedPWM.start(int(currentSetMotorSpeed))
    try:
        while startMotor:
            # Adjust duty cycle if needed
            motorSpeedPWM.ChangeDutyCycle(int(currentSetMotorSpeed))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass  # Allow Ctrl+C to exit the loop
    finally:
        # Stop PWM
        motorSpeedPWM.stop() 

def motorSpeedTimer():
    global startMotor
    global runTimeDurationInput
    global currentSetRunTimeDuration
    print(currentSetRunTimeDuration)
    time.sleep(int(currentSetRunTimeDuration))
    startMotor = False
    startRegulator = False

def pressureRegulatorAdjustment():
    global startPressureRegulator
    global pressureRegulatorPin
    global currentSetPressure
    global startMotor
    startPressureRegulator = True
    i2c = busio.I2C(board.SCL, board.SDA)
    dac = adafruit_mcp4725.MCP4725(i2c)
    while startPressureRegulator:
        dac.raw_value = int(float(currentSetPressure)/15 * 4095)

def manualHardwareControl(switch, index):
    global pickValve_status
    global pin_val
    global manualValveControl
    global lightDarkMode
    global pwmStarted
    global pwm
    global currentManualValveControlState
    # Update current state of manual valve control
    currentManualValveControlState[index + 1] = switch.get()
    # Configure switch colors based on mode and state
    if switch.get() == 1:
        if lightDarkMode.get() == "dark":
            switch.configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1")
        elif lightDarkMode.get() == "light":
            switch.configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9")
    elif switch.get() == 0:
        if lightDarkMode.get() == "dark":
            switch.configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1")
        elif lightDarkMode.get() == "light":
            switch.configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9")
    # Control GPIO pins based on switch and valve status
    if pickValve_status[index + 1].get() == 1:
        if switch.get() == 1:
            if pwmStarted[index + 1] == True:
                pwm[index + 1].stop()
                time.sleep(0.8)
                GPIO.output(pin_val[index], GPIO.HIGH)
                pwmStarted[index + 1] = False
            else:
                GPIO.output(pin_val[index], GPIO.HIGH)
        elif switch.get() == 0:
            if pwmStarted[index + 1] == True:
                pwm[index + 1].stop()
                time.sleep(0.8)
                GPIO.output(pin_val[index], GPIO.LOW)
                pwmStarted[index + 1] = False
            else:
                GPIO.output(pin_val[index], GPIO.LOW)
    elif pickValve_status[index + 1].get() == 0:
        pass
def automaticRunHardwareControl():
    global currentSetFrequency1
    global currentSetFrequency2
    global currentSetFrequency3
    global currentSetFrequency4
    global currentSetDutyCycle1
    global currentSetDutyCycle2
    global currentSetDutyCycle3
    global currentSetDutyCycle4
    global currentSetRunTimeDuration
    global pickValve_status
    global pin_val
    global valveSequencingModeSwitch
    global selected_sequence
    global currentSetTimeInterval
    global differentValveFrequencies
    global differentValveDutyCycle
    global pwm
    global pwmStarted
    global currentSetMotorSpeed
    global startMotor
    global startPressureRegulator
    global currentSetRunTimeDuration
    # Reset PWM settings and flags
    pwm[1] = None
    pwm[2] = None
    pwm[3] = None
    pwm[4] = None
    pwmStarted[1] = False
    pwmStarted[2] = False
    pwmStarted[3] = False
    pwmStarted[4] = False

    # Update lists with current settings
    differentValveFrequencies = [currentSetFrequency1, currentSetFrequency2, currentSetFrequency3, currentSetFrequency4]
    differentValveDutyCycle = [currentSetDutyCycle1, currentSetDutyCycle2, currentSetDutyCycle3, currentSetDutyCycle4]

    # Reset control flags
    startMotor = False
    startPressureRegulator = False

    # Start motor speed adjustment thread if run time duration is set if duration set to 0 or infinity motor will constantly run
    if int(currentSetRunTimeDuration) == 0:
        pass
    else:
        # Start pressure regulator adjustment thread
        motorSpeedTimer_thread = threading.Thread(target=motorSpeedTimer)
        motorSpeedTimer_thread.daemon = True  # Daemonize the thread to allow program exit without waiting for it
        motorSpeedTimer_thread.start()
    
    # Start pressure regulator adjustment thread    
    pressureRegulator_thread = threading.Thread(target=pressureRegulatorAdjustment)
    pressureRegulator_thread.daemon = True  # Daemonize the thread to allow program exit without waiting for it
    pressureRegulator_thread.start()

    # Handle valve control based on sequencing mode
    if valveSequencingModeSwitch.get() == 0:
        for index, valve_num in enumerate(pickValve_status):
            if differentValveFrequencies[index] == 0 and differentValveDutyCycle[index] == 0:
                pass
            else:
                if pickValve_status[valve_num].get() == 1:
                    pwm[index + 1] = GPIO.PWM(pin_val[index], float(differentValveFrequencies[index]))
                    pwm[index + 1].start(int(differentValveDutyCycle[index]))
                    pwmStarted[valve_num] = True 
                elif pickValve_status[valve_num].get() == 0:
                    if pwm[index + 1] is not None:
                        pwm[index + 1].stop()
    elif valveSequencingModeSwitch.get() == 1:
        timeInterval = 0.0
        for char in selected_sequence:
            pwmStarted[int(char)] = True
            if differentValveFrequencies[int(char) - 1] == 0 and differentValveDutyCycle[int(char) - 1] == 0:
                pass
            else:
                pwm[int(char)] =  GPIO.PWM(pin_val[int(char) - 1], float(differentValveFrequencies[int(char) - 1]))
                Timer(timeInterval, startValveSequencing, args=(char)).start()
                timeInterval += round(float(currentSetTimeInterval)/1000, 3)

def startValveSequencing(num):
    global differentValveDutyCycle
    global pwm
    # Start PWM for valve sequencing
    if pwm is not None:
        pwm[int(num)].start(differentValveDutyCycle[int(num) - 1])

def stopHardwareAutomaticControl():
    # Stop hardware automatic control
    global pwm
    global pwmStarted
    global startMotor
    global startPressureRegulator
    startMotor = False
    startPressureRegulator = False
    for valveNum in pwm:
        if pwm[valveNum] is not None:
            pwm[valveNum].stop()
            pwm[valveNum] = None
            pwmStarted[valveNum] = False
        else:
            pass

# live data plot
def liveDataValveStatePlot():
    global pin_val
    global dcLiveDataWindow
    # Create a new window for live data plot
    dcLiveDataWindow = customtkinter.CTkToplevel(root, fg_color = "white")
    dcLiveDataWindow.overrideredirect(True)
    dcLiveDataWindow.geometry("800x480+0+0")

    # Initialize variables for data plotting
    legend_dis = [False]
    x_value = []
    y1_value = []
    y2_value = []
    y3_value = []
    y4_value = []
    counter = count(-1)

    # Set up matplotlib figure and axes
    fig, ax = plt.subplots()
    fig.set_figheight(4)
    fig.set_figwidth(8)
    ax.set_ylim(ymin = -0.1, ymax = 1.1)
    ax.set_xlim(xmin = 0, xmax = 20)
    plt.title("Valve Status Output Over Time")
    plt.xlabel("Time Interval (seconds)")
    plt.ylabel("Valve Status")

    # Animation function to update plot
    def animate(i, legend_displayed, counting):
        # Update data arrays
        if len(x_value) < 22:
            y1_value.append(GPIO.input(pin_val[0]))
            y2_value.append(GPIO.input(pin_val[1]))
            y3_value.append(GPIO.input(pin_val[2]))
            y4_value.append(GPIO.input(pin_val[3]))
            x_value.append(next(counting))
        elif len(x_value) >= 22:
            # Shift data if exceeds window size
            x_valueMin = x_value[0] + 1
            x_valueMax = x_value[-1] + 1
            ax.clear()
            ax.set_ylim(ymin = -0.1, ymax = 1.1)
            ax.set_xlim(xmin=x_valueMin, xmax=x_valueMax)
            y1_value.pop(0)
            y2_value.pop(0)
            y3_value.pop(0)
            y4_value.pop(0)
            x_value.pop(0)
            x_value.append(next(counting))
            y1_value.append(GPIO.input(pin_val[0]))
            y2_value.append(GPIO.input(pin_val[1]))
            y3_value.append(GPIO.input(pin_val[2]))
            y4_value.append(GPIO.input(pin_val[3]))
        
        # Plot data
        plt.title("Valve Status Output Over Time")
        plt.xlabel("Time Interval (ms)")
        plt.ylabel("Valve Status")
        plt.plot(x_value, y1_value, color = 'blue', label = "Valve 1")
        plt.plot(x_value, y2_value, color = 'red', label = "Valve 2")
        plt.plot(x_value, y3_value, color = 'green', label = "Valve 3")
        plt.plot(x_value, y4_value, color = 'orange', label = "Valve 4")

        # Display legend if not already displayed
        if not legend_displayed[0]:
            plt.legend()
            legend_displayed[0] = True

    # Create animation
    ani1 = FuncAnimation(plt.gcf(), animate, fargs = (legend_dis, counter, ), interval=3)
    plt.tight_layout()

    # Create Tkinter canvas for Matplotlib figure
    canvas1 = FigureCanvasTkAgg(fig, master = dcLiveDataWindow)   
    canvas1.draw() 
    canvas1.get_tk_widget().place(relx=0,rely=0.1)

    # Create "Exit Page" button based on current theme
    if lightDarkMode.get() == "dark":
        dcCloseButton = customtkinter.CTkButton(master = dcLiveDataWindow, text="Exit Page", fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1", command= closeValveWindow)
        dcCloseButton.place(relx=0.8,rely=0.05)
    elif lightDarkMode.get() == "light":
        dcCloseButton = customtkinter.CTkButton(master = dcLiveDataWindow, text="Exit Page", fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9", command= closeValveWindow)
        dcCloseButton.place(relx=0.8,rely=0.05) 

# Function to close the live valve status plot window
def closeValveWindow():
    dcLiveDataWindow.destroy()  

# Live data plot for pressure
def liveDataPressurePlot():
    global pressureReader
    global pressureLiveDataWindow
    # Create a new window for live data plot
    pressureLiveDataWindow = customtkinter.CTkToplevel(root, fg_color = "white")
    pressureLiveDataWindow.geometry("800x480+0+0")
    pressureLiveDataWindow.overrideredirect(True)
    pressureLiveDataWindow.wm_attributes("-topmost", True)

    # Initialize variables for data plotting
    x_value = []
    y1_value = []
    y2_value = []
    legend_dis = [False]
    counter = count(-1)

    # Set up matplotlib figure and axes
    fig, ax = plt.subplots()
    fig.set_figheight(4)
    fig.set_figwidth(8)
    ax.set_ylim(ymin = -0.1, ymax = 15.1)
    ax.set_xlim(xmin = 0, xmax = 20)
    plt.title("Pressure Output Over Time")
    plt.xlabel("Time Interval (ms)")
    plt.ylabel("Pressure Output (Psi)")

    # Animation function to update plot
    def animate(i, legend_displayed, counting):
        # Update data arrays
        if len(x_value) < 22:
            y1_value.append(pressureReader[0])
            y2_value.append(pressureReader[1])
            x_value.append(next(counting))
        elif len(x_value) >= 22:
            # Shift data if exceeds window size
            x_valueMin = x_value[0] + 1
            x_valueMax = x_value[-1] + 1
            ax.clear()
            ax.set_ylim(ymin = -0.1, ymax = 15.1)
            ax.set_xlim(xmin=x_valueMin, xmax=x_valueMax)
            y1_value.pop(0)
            y2_value.pop(0)
            x_value.pop(0)
            x_value.append(next(counting))
            y1_value.append(pressureReader[0])
            y2_value.append(pressureReader[1])

        # Plot data
        plt.title("Pressure Output Over Time")
        plt.xlabel("Time Interval (ms)")
        plt.ylabel("Pressure Output (Psi)")
        plt.plot(x_value, y1_value, color = 'blue', label = "Port 1")
        plt.plot(x_value, y2_value, color = 'red', label = "Port 3")

        # Display legend if not already displayed
        if not legend_displayed[0]:
            plt.legend()
            legend_displayed[0] = True
    
    # Create animation
    ani2 = FuncAnimation(plt.gcf(), animate, fargs = (legend_dis, counter,), interval=200)
    plt.tight_layout()
    
    # Create Tkinter canvas for Matplotlib figure
    canvas2 = FigureCanvasTkAgg(fig, master = pressureLiveDataWindow)   
    canvas2.draw() 
    canvas2.get_tk_widget().place(relx=0,rely=0.1)

    # Create "Exit Page" button based on current theme
    if lightDarkMode.get() == "dark":
        pressureCloseButton = customtkinter.CTkButton(master = pressureLiveDataWindow, text="Exit Page", fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1", command= closePressureWindow)
        pressureCloseButton.place(relx=0.8,rely=0.05)
    elif lightDarkMode.get() == "light":
        pressureCloseButton = customtkinter.CTkButton(master = pressureLiveDataWindow, text="Exit Page", fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9", command= closePressureWindow)
        pressureCloseButton.place(relx=0.8,rely=0.05)  

# Function to close the live pressure reading window
def closePressureWindow():
    pressureLiveDataWindow.destroy() 

def incrementFluidControlButton(button, entry):
    global currentSetPressure
    global currentSetMotorSpeed
    global currentSetSequence1 
    global currentSetSequence2 
    global currentSetSequence3 
    global currentSetSequence4 
    global pressure_slider
    global micropump_slider
    current = 0
    # Increment pressure value
    if button == increasePressure:
        if float(entry.get()) < 15:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current + 0.5))
            currentSetPressure = entry.get()
            pressure_slider.set(float(currentSetPressure))
    # Function to increment control method button values
    elif button == increaseSetMotorSpeed:
        if int(entry.get()) < 100:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current + 1))
            currentSetMotorSpeed = entry.get()
            micropump_slider.set(int(currentSetMotorSpeed))

def decrementFluidControlButton(button, entry):
    global currentSetPressure
    global currentSetMotorSpeed
    global currentSetSequence1 
    global currentSetSequence2 
    global currentSetSequence3 
    global currentSetSequence4 
    global pressure_slider
    global micropump_slider
    # Decrement pressure value
    if button == decreasePressure:
        if float(entry.get()) > 0.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current - 0.5))
            currentSetPressure = entry.get()
            pressure_slider.set(float(currentSetPressure))
    # Decrement motor speed value
    elif button == decreaseSetMotorSpeed:
        if int(entry.get()) > 0:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current - 1))
            currentSetMotorSpeed = entry.get()
            micropump_slider.set(int(currentSetMotorSpeed))

def incrementControlMethodButton(button, entry):
    global currentSetFrequency
    global currentSetFrequency1
    global currentSetFrequency2
    global currentSetFrequency3
    global currentSetFrequency4
    global currentSetDutyCycle
    global currentSetDutyCycle1
    global currentSetDutyCycle2
    global currentSetDutyCycle3
    global currentSetDutyCycle4
    global frequency_slider
    global dutycycle_slider
    global frequencySliderValve
    global dutyCycleSliderValve
    global frequencyInputValve
    global dutyCycleInputValve

    # Increment similar frequency value
    if button == increaseSimilarFrequency:
        if float(entry.get()) < 2.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current + 0.2, 1)))
            currentSetFrequency = entry.get()
            frequency_slider.set(float(currentSetFrequency))
            # Update similar frequency values for valves
            currentSetFrequency1 = float(currentSetFrequency)
            currentSetFrequency2 = float(currentSetFrequency)
            currentSetFrequency3 = float(currentSetFrequency)
            currentSetFrequency4 = float(currentSetFrequency)
            frequencyInputValve[1].delete(0, END)
            frequencyInputValve[2].delete(0, END)
            frequencyInputValve[3].delete(0, END)
            frequencyInputValve[4].delete(0, END)
            frequencyInputValve[1].insert(0, currentSetFrequency1)
            frequencyInputValve[2].insert(0, currentSetFrequency2)
            frequencyInputValve[3].insert(0, currentSetFrequency3)
            frequencyInputValve[4].insert(0, currentSetFrequency4)
            frequencySliderValve[1].set(float(currentSetFrequency1))
            frequencySliderValve[2].set(float(currentSetFrequency2))
            frequencySliderValve[3].set(float(currentSetFrequency3))
            frequencySliderValve[4].set(float(currentSetFrequency4))

    # Increment similar duty cycle value
    elif button == increaseSimilarDutyCycle:
        if int(entry.get()) < 100:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current + 10))
            currentSetDutyCycle = entry.get()
            dutycycle_slider.set(int(currentSetDutyCycle))
            # Update similar duty cycle values for valves
            currentSetDutyCycle1 = int(currentSetDutyCycle)
            currentSetDutyCycle2 = int(currentSetDutyCycle)
            currentSetDutyCycle3 = int(currentSetDutyCycle)
            currentSetDutyCycle4 = int(currentSetDutyCycle)
            dutyCycleInputValve[1].delete(0, END)
            dutyCycleInputValve[2].delete(0, END)
            dutyCycleInputValve[3].delete(0, END)
            dutyCycleInputValve[4].delete(0, END)
            dutyCycleInputValve[1].insert(0, currentSetDutyCycle1)
            dutyCycleInputValve[2].insert(0, currentSetDutyCycle2)
            dutyCycleInputValve[3].insert(0, currentSetDutyCycle3)
            dutyCycleInputValve[4].insert(0, currentSetDutyCycle4)
            dutyCycleSliderValve[1].set(int(currentSetDutyCycle1))
            dutyCycleSliderValve[2].set(int(currentSetDutyCycle2))
            dutyCycleSliderValve[3].set(int(currentSetDutyCycle3))
            dutyCycleSliderValve[4].set(int(currentSetDutyCycle4))
    elif button == increaseFrequencyValve[1]:
        if float(entry.get()) < 2.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current + 0.2, 1)))
            currentSetFrequency1 = entry.get()
            frequencySliderValve[1].set(float(currentSetFrequency1))
    elif button == increaseFrequencyValve[2]:
        if float(entry.get()) < 2.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current + 0.2, 1)))
            currentSetFrequency2 = entry.get()
            frequencySliderValve[2].set(float(currentSetFrequency2))
    elif button == increaseFrequencyValve[3]:
        if float(entry.get()) < 2.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current + 0.2, 1)))
            currentSetFrequency3 = entry.get()
            frequencySliderValve[3].set(float(currentSetFrequency3))
    elif button == increaseFrequencyValve[4]:
        if float(entry.get()) < 2.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current + 0.2, 1)))
            currentSetFrequency4 = entry.get()
            frequencySliderValve[4].set(float(currentSetFrequency4))
    elif button == increaseDutyCycleValve[1]:
        if int(entry.get()) < 100:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current + 10))
            currentSetDutyCycle1 = entry.get()
            dutyCycleSliderValve[1].set(int(currentSetDutyCycle1))
    elif button == increaseDutyCycleValve[2]:
        if int(entry.get()) < 100:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current + 10))
            currentSetDutyCycle2 = entry.get()
            dutyCycleSliderValve[2].set(int(currentSetDutyCycle2))
    elif button == increaseDutyCycleValve[3]:
        if int(entry.get()) < 100:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current + 10))
            currentSetDutyCycle3 = entry.get()
            dutyCycleSliderValve[3].set(int(currentSetDutyCycle3))
    elif button == increaseDutyCycleValve[4]:
        if int(entry.get()) < 100:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current + 10))
            currentSetDutyCycle4 = entry.get()
            dutyCycleSliderValve[4].set(int(currentSetDutyCycle4))

def decrementControlMethodButton(button, entry):
    global currentSetFrequency1
    global currentSetFrequency2
    global currentSetFrequency3
    global currentSetFrequency4
    global currentSetDutyCycle1
    global currentSetDutyCycle2
    global currentSetDutyCycle3
    global currentSetDutyCycle4
    global frequency_slider
    global dutycycle_slider
    global frequencySliderValve
    global dutyCycleSliderValve
    global frequencyInputValve
    global dutyCycleInputValve
    # Decrement similar frequency value
    if button == decreaseSimilarFrequency:
        if float(entry.get()) > 0.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current - 0.2, 1)))
            currentSetFrequency = entry.get()
            frequency_slider.set(float(currentSetFrequency))
            # Update similar frequency values for valves            
            currentSetFrequency1 = float(currentSetFrequency)
            currentSetFrequency2 = float(currentSetFrequency)
            currentSetFrequency3 = float(currentSetFrequency)
            currentSetFrequency4 = float(currentSetFrequency)
            frequencyInputValve[1].delete(0, END)
            frequencyInputValve[2].delete(0, END)
            frequencyInputValve[3].delete(0, END)
            frequencyInputValve[4].delete(0, END)
            frequencyInputValve[1].insert(0, currentSetFrequency1)
            frequencyInputValve[2].insert(0, currentSetFrequency2)
            frequencyInputValve[3].insert(0, currentSetFrequency3)
            frequencyInputValve[4].insert(0, currentSetFrequency4)
            frequencySliderValve[1].set(float(currentSetFrequency1))
            frequencySliderValve[2].set(float(currentSetFrequency2))
            frequencySliderValve[3].set(float(currentSetFrequency3))
            frequencySliderValve[4].set(float(currentSetFrequency4))

    # Decrement similar duty cycle value    
    if button == decreaseSimilarDutyCycle:
        if int(entry.get()) > 0:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current - 10))
            currentSetDutyCycle = entry.get()
            dutycycle_slider.set(int(currentSetDutyCycle))

            # Update similar duty cycle values for valves
            currentSetDutyCycle1 = int(currentSetDutyCycle)
            currentSetDutyCycle2 = int(currentSetDutyCycle)
            currentSetDutyCycle3 = int(currentSetDutyCycle)
            currentSetDutyCycle4 = int(currentSetDutyCycle)
            dutyCycleInputValve[1].delete(0, END)
            dutyCycleInputValve[2].delete(0, END)
            dutyCycleInputValve[3].delete(0, END)
            dutyCycleInputValve[4].delete(0, END)
            dutyCycleInputValve[1].insert(0, currentSetDutyCycle1)
            dutyCycleInputValve[2].insert(0, currentSetDutyCycle2)
            dutyCycleInputValve[3].insert(0, currentSetDutyCycle3)
            dutyCycleInputValve[4].insert(0, currentSetDutyCycle4)
            dutyCycleSliderValve[1].set(int(currentSetDutyCycle1))
            dutyCycleSliderValve[2].set(int(currentSetDutyCycle2))
            dutyCycleSliderValve[3].set(int(currentSetDutyCycle3))
            dutyCycleSliderValve[4].set(int(currentSetDutyCycle4))
    elif button == decreaseFrequencyValve[1]:
        if float(entry.get()) > 0.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current - 0.2, 1)))
            currentSetFrequency1 = entry.get()
            frequencySliderValve[1].set(float(currentSetFrequency1))
    elif button == decreaseFrequencyValve[2]:
        if float(entry.get()) > 0.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current - 0.2, 1)))
            currentSetFrequency2 = entry.get()
            frequencySliderValve[2].set(float(currentSetFrequency2))
    elif button == decreaseFrequencyValve[3]:
        if float(entry.get()) > 0.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current - 0.2, 1)))
            currentSetFrequency3 = entry.get()
            frequencySliderValve[3].set(float(currentSetFrequency3))
    elif button == decreaseFrequencyValve[4]:
        if float(entry.get()) > 0.0:
            current = float(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(round(current - 0.2, 1)))
            currentSetFrequency4 = entry.get()
            frequencySliderValve[4].set(float(currentSetFrequency4))
    elif button == decreaseDutyCycleValve[1]:
        if int(entry.get()) > 0:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current - 10))
            currentSetDutyCycle1 = entry.get()
            dutyCycleSliderValve[1].set(int(currentSetDutyCycle1))
    elif button == decreaseDutyCycleValve[2]:
        if int(entry.get()) > 0:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current - 10))
            currentSetDutyCycle2 = entry.get()
            dutyCycleSliderValve[2].set(int(currentSetDutyCycle2))
    elif button == decreaseDutyCycleValve[3]:
        if int(entry.get()) > 0:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current - 10))
            currentSetDutyCycle3 = entry.get()
            dutyCycleSliderValve[3].set(int(currentSetDutyCycle3))
    elif button == decreaseDutyCycleValve[4]:
        if int(entry.get()) > 0:
            current = int(entry.get())
            entry.delete(0, END)
            entry.insert(0, str(current - 10))
            currentSetDutyCycle4 = entry.get()
            dutyCycleSliderValve[4].set(int(currentSetDutyCycle4))
def microPumpSlider(value):
    global currentSetMotorSpeed
    global motorSpeedInput
    global micropump_slider
    # Update micro pump motor speed value based on the slider value
    currentSetMotorSpeed = micropump_slider.get()
    # Clear the current content in the motor speed input field and update it with the new motor speed value
    motorSpeedInput.delete(0, END)
    motorSpeedInput.insert(0, str(int(currentSetMotorSpeed)))
def pressureSlider(value):
    global currentSetPressure
    global pressureInput
    global pressure_slider
    # Update the current pressure value based on the slider value
    currentSetPressure = pressure_slider.get()
    # Clear the current content in the pressure input field and update it with the new pressure value
    pressureInput.delete(0, END)
    pressureInput.insert(0, str(currentSetPressure))

# First page of the GUI fluid Control main page
def fluidControl_page():
    global main_frame
    global setPressure
    global pressureInput
    global pressureUnit
    global setMotorSpeed
    global motorSpeedInput
    global motorSpeedUnit
    global increasePressure
    global decreasePressure
    global increaseSetMotorSpeed
    global decreaseSetMotorSpeed
    global pressure_slider
    global micropump_slider
    global currentSetPressure
    global currentSetMotorSpeed
    global pressureInput
    global motorSpeedInput
    global lightDarkMode
    # create new main frame 
    main_frame["fluidControl_page"] = customtkinter.CTkFrame(root, width = 450, height = 480)
    main_frame["fluidControl_page"].grid(row = 0, column = 1, rowspan = 4, padx = 5, sticky = "NSWE")
    main_frame["fluidControl_page"].grid_propagate(0)

    # create new pressure frame 
    main_frame["pressure"] = customtkinter.CTkFrame(main_frame["fluidControl_page"], width = 440, height = 100)
    main_frame["pressure"].grid(row = 0, column = 0, padx = 5, pady = (10, 0), sticky = "NSWE")
    main_frame["pressure"].grid_propagate(0)

    # label set pressure
    setPressure = customtkinter.CTkLabel(main_frame["pressure"], text = "Set Pressure          : ", font = ('SF Compact', 15, 'bold'), anchor = "w")
    setPressure.grid(row = 0, column = 0, padx = (15, 0), pady = (15, 0), sticky = "NSWE")

    # user input pressure
    pressureInput = customtkinter.CTkEntry(main_frame["pressure"], width = 50, font = ('SF Compact', 15, 'bold'), justify = "center")
    pressureInput.grid(row = 0, column = 1, columnspan = 3, padx = (15, 0), pady = (15, 0), sticky = "W")

    # pressure unit
    pressureUnit = customtkinter.CTkLabel(main_frame["pressure"], text = "Psi", font = ('SF Compact', 15, 'bold'))
    pressureUnit.grid(row = 0, column = 2, padx = (15, 0), pady = (15, 0), sticky = "W")

    # set current pressure in pressure input box
    pressureInput.insert(0, str(currentSetPressure))

    # decrement pressure button
    decreasePressure = customtkinter.CTkButton(main_frame["pressure"], text = "- 0.5", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementFluidControlButton(decreasePressure, pressureInput))
    decreasePressure.grid(row = 1, column = 1, padx = (15, 0), pady = (10, 0), sticky = "W")

    # increment pressure button
    increasePressure = customtkinter.CTkButton(main_frame["pressure"], text = "+ 0.5", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementFluidControlButton(increasePressure, pressureInput))
    increasePressure.grid(row = 1, column = 3, pady = (10, 0), sticky = "W")

    # pressure sliding bar
    pressure_slider = customtkinter.CTkSlider(main_frame["pressure"], width = 100, from_ = 0.0, to = 15.0, number_of_steps = 30, command = pressureSlider)
    pressure_slider.grid(row = 1, column = 2, padx = 10, pady = (10, 0), sticky = "W")
    pressure_slider.set(float(currentSetPressure))

    # create new micropump frame 
    main_frame["motorSpeed"] = customtkinter.CTkFrame(main_frame["fluidControl_page"], width = 440, height = 100)
    main_frame["motorSpeed"].grid(row = 1, column = 0, padx = 5, pady = (10, 0), sticky = "NSWE")
    main_frame["motorSpeed"].grid_propagate(0)

    # label set motor speed
    setMotorSpeed = customtkinter.CTkLabel(main_frame["motorSpeed"], text = "Micropump Speed : ", font = ('SF Compact', 15, 'bold'), anchor = "w")
    setMotorSpeed.grid(row = 0, column = 0, padx = (15, 0), pady = (15, 0), sticky = "NSWE")

    # user input motor speed
    motorSpeedInput = customtkinter.CTkEntry(main_frame["motorSpeed"], width = 50, font = ('SF Compact', 15, 'bold'), justify = "center")
    motorSpeedInput.grid(row = 0, column = 1, columnspan = 3, padx = (15, 0), pady = (15, 0), sticky = "W")

    # set current motor speed in motor speed input box
    motorSpeedInput.insert(0, str(currentSetMotorSpeed))

    # motor speed unit
    motorSpeedUnit = customtkinter.CTkLabel(main_frame["motorSpeed"], text = "   %", font = ('SF Compact', 15, 'bold'))
    motorSpeedUnit.grid(row = 0, column = 2, padx = (15, 0), pady = (15, 0), sticky = "W")

    # decrement micro pump speed button
    decreaseSetMotorSpeed = customtkinter.CTkButton(main_frame["motorSpeed"], text = "- 1", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementFluidControlButton(decreaseSetMotorSpeed, motorSpeedInput))
    decreaseSetMotorSpeed.grid(row = 1, column = 1, padx = (15, 0), pady = (10, 0), sticky = "W")

    # increment pressure button
    increaseSetMotorSpeed = customtkinter.CTkButton(main_frame["motorSpeed"], text = "+ 1", font = ('SF Compact', 15, 'bold'), width = 5,  command = lambda: incrementFluidControlButton(increaseSetMotorSpeed, motorSpeedInput))
    increaseSetMotorSpeed.grid(row = 1, column = 3, pady = (10, 0), sticky = "W")

    # pressure sliding bar
    micropump_slider = customtkinter.CTkSlider(main_frame["motorSpeed"], width = 100, from_ = 0, to = 100, number_of_steps = 101, command = microPumpSlider)
    micropump_slider.grid(row = 1, column = 2, padx = 10, pady = (10, 0), sticky = "W")
    micropump_slider.set(int(currentSetMotorSpeed))  
    if lightDarkMode.get() == "light":
        main_frame["fluidControl_page"].configure(fg_color = "#F0F4C3")
        main_frame["pressure"].configure(fg_color = "#F9FBE7")
        main_frame["motorSpeed"].configure(fg_color = "#F9FBE7")
        setPressure.configure(fg_color = "#F9FBE7",text_color = "black")
        pressureInput.configure(fg_color = "#F9FBE7", text_color = "black")
        pressureUnit.configure(fg_color = "#F9FBE7", text_color = "black")
        decreasePressure.configure(fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9")
        increasePressure.configure(fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9")
        pressure_slider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_hover_color = "#E8F5E9")
        setMotorSpeed.configure(fg_color = "#F9FBE7", text_color = "black")
        motorSpeedInput.configure(fg_color = "#F9FBE7", text_color = "black")
        motorSpeedUnit.configure(fg_color = "#F9FBE7", text_color = "black")
        decreaseSetMotorSpeed.configure(fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9")
        increaseSetMotorSpeed.configure(fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9")
        micropump_slider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_hover_color = "#E8F5E9")
    elif lightDarkMode.get() == "dark":
        main_frame["fluidControl_page"].configure(fg_color = "#2F2F2F")
        main_frame["pressure"].configure(fg_color = "#3F3F3F")
        main_frame["motorSpeed"].configure(fg_color = "#3F3F3F")
        setPressure.configure(fg_color = "#3F3F3F", text_color = "white")
        pressureInput.configure(fg_color = "#3F3F3F", text_color = "white")
        pressureUnit.configure(fg_color = "#3F3F3F", text_color = "white")
        decreasePressure.configure(fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1")
        increasePressure.configure(fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1")
        pressure_slider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        setMotorSpeed.configure(fg_color = "#3F3F3F", text_color = "white")
        motorSpeedInput.configure(fg_color = "#3F3F3F", text_color = "white")
        motorSpeedUnit.configure(fg_color = "#3F3F3F", text_color = "white")
        decreaseSetMotorSpeed.configure(fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1")
        increaseSetMotorSpeed.configure(fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1")
        micropump_slider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")

def similarFrequencySlider(value):
    global currentSetFrequency1
    global currentSetFrequency2
    global currentSetFrequency3
    global currentSetFrequency
    global frequencyInputValve
    global frequencySliderValve
    global currentSetFrequency
    global similarFrequencyInput
    global frequency_slider
    # Get the current value of the frequency slider
    currentSetFrequency = round(frequency_slider.get(), 1)

    # Clear previous values from input fields
    similarFrequencyInput.delete(0, END)
    frequencyInputValve[1].delete(0, END)
    frequencyInputValve[2].delete(0, END)
    frequencyInputValve[3].delete(0, END)
    frequencyInputValve[4].delete(0, END)

    # Insert the current frequency value into input fields
    similarFrequencyInput.insert(0, str(currentSetFrequency))
    currentSetFrequency1 = currentSetFrequency
    currentSetFrequency2 = currentSetFrequency
    currentSetFrequency3 = currentSetFrequency
    currentSetFrequency4 = currentSetFrequency
    frequencyInputValve[1].insert(0, str(currentSetFrequency1))
    frequencyInputValve[2].insert(0, str(currentSetFrequency2))
    frequencyInputValve[3].insert(0, str(currentSetFrequency3))
    frequencyInputValve[4].insert(0, str(currentSetFrequency4))

    # Set the slider values to match the current frequency
    frequencySliderValve[1].set(currentSetFrequency1)
    frequencySliderValve[2].set(currentSetFrequency2)
    frequencySliderValve[3].set(currentSetFrequency3)
    frequencySliderValve[4].set(currentSetFrequency4)
def similarDutyCycleSlider(value):
    global currentSetDutyCycle1
    global currentSetDutyCycle2
    global currentSetDutyCycle3
    global currentSetDutyCycle4
    global dutyCycleInputValve
    global dutyCycleSliderValve
    global currentSetDutyCycle
    global similarDutyCycleInput
    global dutycycle_slider
    # Get the current value of the duty cycle slider
    currentSetDutyCycle = dutycycle_slider.get()

    # Clear previous values from input fields
    similarDutyCycleInput.delete(0, END)
    dutyCycleInputValve[1].delete(0, END)
    dutyCycleInputValve[2].delete(0, END)
    dutyCycleInputValve[3].delete(0, END)
    dutyCycleInputValve[4].delete(0, END)

    # Insert the current duty cycle value into input fields
    similarDutyCycleInput.insert(0, str(int(currentSetDutyCycle)))
    currentSetDutyCycle1 = int(currentSetDutyCycle)
    currentSetDutyCycle2 = int(currentSetDutyCycle)
    currentSetDutyCycle3 = int(currentSetDutyCycle)
    currentSetDutyCycle4 = int(currentSetDutyCycle)
    dutyCycleInputValve[1].insert(0, str(int(currentSetDutyCycle1)))
    dutyCycleInputValve[2].insert(0, str(int(currentSetDutyCycle2)))
    dutyCycleInputValve[3].insert(0, str(int(currentSetDutyCycle3)))
    dutyCycleInputValve[4].insert(0, str(int(currentSetDutyCycle4)))

    # Set the slider values to match the current duty cycle
    dutyCycleSliderValve[1].set(int(currentSetDutyCycle1))
    dutyCycleSliderValve[2].set(int(currentSetDutyCycle2))
    dutyCycleSliderValve[3].set(int(currentSetDutyCycle3))
    dutyCycleSliderValve[4].set(int(currentSetDutyCycle4))

def uniqueFrequencySlider(value, slider_id):
    global currentSetFrequency1
    global currentSetFrequency2
    global currentSetFrequency3
    global currentSetFrequency4
    global frequencyInputValve
    global frequencySliderValve
    # Check which slider is being adjusted and update the corresponding frequency value
    if slider_id == 1:
        currentSetFrequency1 = round(frequencySliderValve[1].get(), 1)
        frequencyInputValve[1].delete(0, END)
        frequencyInputValve[1].insert(0, str(currentSetFrequency1))
    elif slider_id == 2:
        currentSetFrequency2 = round(frequencySliderValve[2].get(), 1)
        frequencyInputValve[2].delete(0, END)
        frequencyInputValve[2].insert(0, str(currentSetFrequency2))
    elif slider_id == 3:
        currentSetFrequency3 = round(frequencySliderValve[3].get(), 1)
        frequencyInputValve[3].delete(0, END)
        frequencyInputValve[3].insert(0, str(currentSetFrequency3))
    elif slider_id == 4:
        currentSetFrequency4 = round(frequencySliderValve[4].get(), 1)
        frequencyInputValve[4].delete(0, END)
        frequencyInputValve[4].insert(0, str(currentSetFrequency4))

def uniqueDutyCycleSlider(value, slider_id):
    global currentSetDutyCycle1
    global currentSetDutyCycle2
    global currentSetDutyCycle3
    global currentSetDutyCycle4
    global dutyCycleInputValve
    global dutyCycleSliderValve
    # Check which slider is being adjusted and update the corresponding frequency value
    if slider_id == 1:
        currentSetDutyCycle1 = dutyCycleSliderValve[1].get()
        dutyCycleInputValve[1].delete(0, END)
        dutyCycleInputValve[1].insert(0, str(int(currentSetDutyCycle1)))
    elif slider_id == 2:
        currentSetDutyCycle2 = dutyCycleSliderValve[2].get()
        dutyCycleInputValve[2].delete(0, END)
        dutyCycleInputValve[2].insert(0, str(int(currentSetDutyCycle2)))
    elif slider_id == 3:
        currentSetDutyCycle3 = dutyCycleSliderValve[3].get()
        dutyCycleInputValve[3].delete(0, END)
        dutyCycleInputValve[3].insert(0, str(int(currentSetDutyCycle3)))
    elif slider_id == 4:
        currentSetDutyCycle4 = dutyCycleSliderValve[4].get()
        dutyCycleInputValve[4].delete(0, END)
        dutyCycleInputValve[4].insert(0, str(int(currentSetDutyCycle4)))

# control method page of the GUI
def controlMethod_page():
    global main_frame
    global currentValves_status
    global manualButton
    global frequencyInput
    global dutyCycleInput
    global valveIndicator
    global increaseSimilarFrequency
    global decreaseSimilarFrequency
    global increaseSimilarDutyCycle
    global decreaseSimilarDutyCycle
    global increaseFrequency
    global decreaseFrequency
    global increaseDutyCycle
    global decreaseDutyCycle
    global currentSetFrequency
    global currentSetFrequency1
    global currentSetFrequency2
    global currentSetFrequency3
    global currentSetFrequency4
    global currentSetDutyCycle
    global currentSetDutyCycle1
    global currentSetDutyCycle2
    global currentSetDutyCycle3
    global currentSetDutyCycle4
    global similarFrequencyInput
    global similarDutyCycleInput
    global manualValveControl
    global frequencyInputValve
    global frequencySliderValve
    global dutyCycleInputValve
    global dutyCycleSliderValve
    global decreaseDutyCycleValve
    global increaseDutyCycleValve
    global decreaseFrequencyValve
    global increaseFrequencyValve
    global frequency_slider 
    global dutycycle_slider 
    global manualControlLabel
    global autoControlLabel
    global manualValveControl
    global similarFrequencyLabel
    global similarFrequencyInput
    global similarFrequencyUnit
    global similarDutyCycleLabel
    global similarDutyCycleInput
    global similarDutyCycleUnit
    global valve1Label
    global valve2Label
    global valve3Label
    global valve4Label
    global frequency1Label
    global frequency2Label
    global frequency3Label
    global frequency4Label
    global frequency1Unit
    global frequency2Unit
    global frequency3Unit
    global frequency4Unit
    global dutyCycleValve1Label
    global dutyCycleValve2Label
    global dutyCycleValve3Label
    global dutyCycleValve4Label
    global dutyCycleValve1Unit
    global dutyCycleValve2Unit
    global dutyCycleValve3Unit
    global dutyCycleValve4Unit
    global SequencingOrderLabel
    global selectSequenceOrderInput
    global timeIntervalLabel
    global timeIntervalInput
    global timeIntervalUnit
    global timeIntervalSlider
    global decreaseTimeInterval
    global increaseTimeInterval
    global lightDarkMode
    global currentManualValveControlState
    # create new main frame
    main_frame["controlMethod"] = customtkinter.CTkFrame(root, width = 450, height = 480)
    main_frame["controlMethod"].grid(row = 0, column = 1, rowspan = 4, padx = 5, sticky = "NSWE")
    main_frame["controlMethod"].grid_propagate(0)

    # create manual control frame
    main_frame["manual_frame"] = customtkinter.CTkFrame(main_frame["controlMethod"], width = 418, height = 90)
    main_frame["manual_frame"].grid(row = 0, column = 0, padx = 5, pady = (10, 0), sticky = "NSWE")
    main_frame["manual_frame"].grid_propagate(0)

    # create automatic control frame
    main_frame["auto_frame"] = customtkinter.CTkScrollableFrame(main_frame["controlMethod"], orientation = "vertical", width = 418, height = 350)
    main_frame["auto_frame"].grid(row = 1, column = 0, padx = 5, pady = (10, 0), sticky = "NSWE")
    # manual control label
    manualControlLabel = customtkinter.CTkLabel(main_frame["manual_frame"], text = "Manual Control", font = ('Courier New', 14, 'bold'))
    manualControlLabel.grid(row = 0, column = 0, padx = (15, 0), pady = (1, 0), sticky = "W")

    # automatic control label
    autoControlLabel = customtkinter.CTkLabel(main_frame["auto_frame"], text = "Automatic Control", font = ('Courier New', 14, 'bold'))
    autoControlLabel.grid(row = 0, column = 0, padx = (15, 0), pady = (1, 0), sticky = "W")

    # create manual switch for each valve
    manualValveControl[1] = customtkinter.CTkSwitch(main_frame["manual_frame"], text = "Valve 1", command = lambda: manualHardwareControl(manualValveControl[1], 0))
    manualValveControl[1].grid(row = 1, column = 0, padx = (15, 0), pady = (1, 0))
    if currentManualValveControlState[1] == 0:
        manualValveControl[1].deselect()
    elif currentManualValveControlState[1] == 1:
        manualValveControl[1].select()
    manualValveControl[2] = customtkinter.CTkSwitch(main_frame["manual_frame"], text = "Valve 2", command = lambda: manualHardwareControl(manualValveControl[2], 1))
    manualValveControl[2].grid(row = 1, column = 1, padx = (50, 0), pady = (1, 0))
    if currentManualValveControlState[2] == 0:
        manualValveControl[2].deselect()
    elif currentManualValveControlState[2] == 1:
        manualValveControl[2].select()
    manualValveControl[3] = customtkinter.CTkSwitch(main_frame["manual_frame"], text = "Valve 3", command = lambda: manualHardwareControl(manualValveControl[3], 2))
    manualValveControl[3].grid(row = 2, column = 0, padx = (15, 0), pady = (1, 0))
    if currentManualValveControlState[3] == 0:
        manualValveControl[3].deselect()
    elif currentManualValveControlState[3] == 1:
        manualValveControl[3].select()
    manualValveControl[4] = customtkinter.CTkSwitch(main_frame["manual_frame"], text = "Valve 4", command = lambda: manualHardwareControl(manualValveControl[4], 3))
    manualValveControl[4].grid(row = 2, column = 1, padx = (50, 0), pady = (1, 0))
    if currentManualValveControlState[4] == 0:
        manualValveControl[4].deselect()
    elif currentManualValveControlState[4] == 1:
        manualValveControl[4].select()
    main_frame["setSimilarInputFrame"] = customtkinter.CTkFrame(main_frame["auto_frame"], width = 410, height = 140)
    main_frame["setSimilarInputFrame"].grid(row = 1, column = 0, padx = 5, pady = (5, 0), sticky = "NSWE")
    main_frame["setSimilarInputFrame"].grid_propagate(0)

    # label automatic valves to set equal frequency for all valves
    similarFrequencyLabel = customtkinter.CTkLabel(main_frame["setSimilarInputFrame"], text = "Equal Frequency  :", font = ('Courier New', 14, 'bold'))
    similarFrequencyLabel.grid(row = 0, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")
    similarFrequencyInput = customtkinter.CTkEntry(main_frame["setSimilarInputFrame"], width = 70, font = ('SF Compact', 15, 'bold'), justify = "center")
    similarFrequencyInput.grid(row = 0, column = 1, columnspan = 3, padx = (15, 0), pady = (5, 0), sticky = "W")
    similarFrequencyUnit = customtkinter.CTkLabel(main_frame["setSimilarInputFrame"], text = "Hz", font = ('Courier New', 14, 'bold'))
    similarFrequencyUnit.grid(row = 0, column = 2, padx = (50, 0), pady = (5, 0), sticky = "W")
    similarFrequencyInput.insert(0, str(currentSetFrequency))

    # frequency sliding bar for all valves
    frequency_slider = customtkinter.CTkSlider(main_frame["setSimilarInputFrame"], width = 100, from_ = 0.0, to = 2.0, number_of_steps = 10, command = similarFrequencySlider)
    frequency_slider.grid(row = 1, column = 2, padx = 10, pady = (5, 0), sticky = "W")
    frequency_slider.set(float(currentSetFrequency))

    # decrement frequency button for all valves
    decreaseSimilarFrequency = customtkinter.CTkButton(main_frame["setSimilarInputFrame"], text = "- 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseSimilarFrequency, similarFrequencyInput))
    decreaseSimilarFrequency.grid(row = 1, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")

    # increment frequency button for all valves
    increaseSimilarFrequency = customtkinter.CTkButton(main_frame["setSimilarInputFrame"], text = "+ 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseSimilarFrequency, similarFrequencyInput))
    increaseSimilarFrequency.grid(row = 1, column = 3, pady = (5, 0), sticky = "W")

    # label automatic valves to set equal duty cycle for all valves
    similarDutyCycleLabel = customtkinter.CTkLabel(main_frame["setSimilarInputFrame"], text = "Equal Duty Cycle :", font = ('Courier New', 14, 'bold'))
    similarDutyCycleLabel.grid(row = 2, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")
    similarDutyCycleInput = customtkinter.CTkEntry(main_frame["setSimilarInputFrame"], width = 70, font = ('SF Compact', 15, 'bold'), justify = "center")
    similarDutyCycleInput.grid(row = 2, column = 1, columnspan = 3, padx = (15, 0), pady = (5, 0), sticky = "W")
    similarDutyCycleUnit = customtkinter.CTkLabel(main_frame["setSimilarInputFrame"], text = "%", font = ('Courier New', 14, 'bold'))
    similarDutyCycleUnit.grid(row = 2, column = 2, padx = (50, 0), pady = (5, 0), sticky = "W")
    similarDutyCycleInput.insert(0, str(int(currentSetDutyCycle)))

    # duty cycle sliding bar for all valves
    dutycycle_slider = customtkinter.CTkSlider(main_frame["setSimilarInputFrame"], width = 100, from_ = 0, to = 100, number_of_steps = 10, command = similarDutyCycleSlider)
    dutycycle_slider.grid(row = 3, column = 2, padx = 10, pady = 10, sticky = "W")
    dutycycle_slider.set(int(currentSetDutyCycle))

    # decrement duty cycle button for all valves
    decreaseSimilarDutyCycle = customtkinter.CTkButton(main_frame["setSimilarInputFrame"], text = "- 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseSimilarDutyCycle, similarDutyCycleInput))
    decreaseSimilarDutyCycle.grid(row = 3, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")

    # increment duty cycle button for all valves
    increaseSimilarDutyCycle = customtkinter.CTkButton(main_frame["setSimilarInputFrame"], text = "+ 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseSimilarDutyCycle, similarDutyCycleInput))
    increaseSimilarDutyCycle.grid(row = 3, column = 3, pady = (5, 0), sticky = "W")
    
    # unique input for frequency and duty cycle
    main_frame["setUniqueInputFrame"] = customtkinter.CTkFrame(main_frame["auto_frame"], width = 410, height = 650)
    main_frame["setUniqueInputFrame"].grid(row = 2, column = 0, padx = 5, pady = (10, 0), sticky = "NSWE")
    main_frame["setUniqueInputFrame"].grid_propagate(0)

    # individual frequency and duty cycle set by the user
    valve1Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = f"Valve 1", font = ('Courier New', 14, 'bold'))
    valve1Label.grid(row = 0, column = 0, padx = (15, 0), sticky = "W")
    valve2Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = f"Valve 2", font = ('Courier New', 14, 'bold'))
    valve2Label.grid(row = 5, column = 0, padx = (15, 0), sticky = "W")
    valve3Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = f"Valve 3", font = ('Courier New', 14, 'bold'))
    valve3Label.grid(row = 10, column = 0, padx = (15, 0), sticky = "W")
    valve4Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = f"Valve 4", font = ('Courier New', 14, 'bold'))
    valve4Label.grid(row = 15, column = 0, padx = (15, 0), sticky = "W")
    frequency1Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Frequency :", font = ('Courier New', 14, 'bold'))
    frequency1Label.grid(row = 1, column = 0, padx = (15, 0), sticky = "W")
    frequency2Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Frequency :", font = ('Courier New', 14, 'bold'))
    frequency2Label.grid(row = 6, column = 0, padx = (15, 0), sticky = "W")
    frequency3Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Frequency :", font = ('Courier New', 14, 'bold'))
    frequency3Label.grid(row = 11, column = 0, padx = (15, 0), sticky = "W")
    frequency4Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Frequency :", font = ('Courier New', 14, 'bold'))
    frequency4Label.grid(row = 16, column = 0, padx = (15, 0), sticky = "W")
    frequencyInputValve[1] = customtkinter.CTkEntry(main_frame["setUniqueInputFrame"], width = 70, font = ('SF Compact', 14, 'bold'), justify = "center")
    frequencyInputValve[1].grid(row = 1, column = 1, columnspan = 3, padx = (15, 0), sticky = "W")
    frequencyInputValve[1].insert(0, str(currentSetFrequency1))
    frequencyInputValve[2] = customtkinter.CTkEntry(main_frame["setUniqueInputFrame"], width = 70, font = ('SF Compact', 14, 'bold'), justify = "center")
    frequencyInputValve[2].grid(row = 6, column = 1, columnspan = 3, padx = (15, 0), sticky = "W")
    frequencyInputValve[2].insert(0, str(currentSetFrequency2))
    frequencyInputValve[3] = customtkinter.CTkEntry(main_frame["setUniqueInputFrame"], width = 70, font = ('SF Compact', 14, 'bold'), justify = "center")
    frequencyInputValve[3].grid(row = 11, column = 1, columnspan = 3, padx = (15, 0), sticky = "W")
    frequencyInputValve[3].insert(0, str(currentSetFrequency3))
    frequencyInputValve[4] = customtkinter.CTkEntry(main_frame["setUniqueInputFrame"], width = 70, font = ('SF Compact', 14, 'bold'), justify = "center")
    frequencyInputValve[4].grid(row = 16, column = 1, columnspan = 3, padx = (15, 0), sticky = "W")
    frequencyInputValve[4].insert(0, str(currentSetFrequency4))
    frequency1Unit = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Hz", font = ('Courier New', 14, 'bold'))
    frequency1Unit.grid(row = 1, column = 2, padx = (50, 0), sticky = "W")
    frequency2Unit = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Hz", font = ('Courier New', 14, 'bold'))
    frequency2Unit.grid(row = 6, column = 2, padx = (50, 0), sticky = "W")
    frequency3Unit = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Hz", font = ('Courier New', 14, 'bold'))
    frequency3Unit.grid(row = 11, column = 2, padx = (50, 0), sticky = "W")
    frequency4Unit = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Hz", font = ('Courier New', 14, 'bold'))
    frequency4Unit.grid(row = 16, column = 2, padx = (50, 0), sticky = "W")

    # frequency sliding bar for each valve
    frequencySliderValve[1] = customtkinter.CTkSlider(main_frame["setUniqueInputFrame"], width = 100, from_ = 0.0, to = 2.0, number_of_steps = 10, command = lambda value: uniqueFrequencySlider(value, 1))
    frequencySliderValve[1].grid(row = 2, column = 2, padx = 10, pady = (5, 0), sticky = "W")
    frequencySliderValve[1].set(float(currentSetFrequency1))
    frequencySliderValve[2] = customtkinter.CTkSlider(main_frame["setUniqueInputFrame"], width = 100, from_ = 0.0, to = 2.0, number_of_steps = 10, command = lambda value: uniqueFrequencySlider(value, 2))
    frequencySliderValve[2].grid(row = 7, column = 2, padx = 10, pady = (5, 0), sticky = "W")
    frequencySliderValve[2].set(float(currentSetFrequency2))
    frequencySliderValve[3] = customtkinter.CTkSlider(main_frame["setUniqueInputFrame"], width = 100, from_ = 0.0, to = 2.0, number_of_steps = 10, command = lambda value: uniqueFrequencySlider(value, 3))
    frequencySliderValve[3].grid(row = 12, column = 2, padx = 10, pady = (5, 0), sticky = "W")
    frequencySliderValve[3].set(float(currentSetFrequency3))
    frequencySliderValve[4] = customtkinter.CTkSlider(main_frame["setUniqueInputFrame"], width = 100, from_ = 0.0, to = 2.0, number_of_steps = 10, command = lambda value: uniqueFrequencySlider(value, 4))
    frequencySliderValve[4].grid(row = 17, column = 2, padx = 10, pady = (5, 0), sticky = "W")
    frequencySliderValve[4].set(float(currentSetFrequency4))

    # decrement frequency button for each valve
    decreaseFrequencyValve[1] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "- 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseFrequencyValve[1], frequencyInputValve[1]))
    decreaseFrequencyValve[1].grid(row = 2, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")
    decreaseFrequencyValve[2] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "- 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseFrequencyValve[2],frequencyInputValve[2]))
    decreaseFrequencyValve[2].grid(row = 7, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")
    decreaseFrequencyValve[3] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "- 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseFrequencyValve[3], frequencyInputValve[3]))
    decreaseFrequencyValve[3].grid(row = 12, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")
    decreaseFrequencyValve[4] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "- 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseFrequencyValve[4], frequencyInputValve[4]))
    decreaseFrequencyValve[4].grid(row = 17, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")

    # increment frequency button for each valve
    increaseFrequencyValve[1] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "+ 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseFrequencyValve[1], frequencyInputValve[1]))
    increaseFrequencyValve[1].grid(row = 2, column = 3, pady = (5, 0), sticky = "W")
    increaseFrequencyValve[2] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "+ 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseFrequencyValve[2], frequencyInputValve[2]))
    increaseFrequencyValve[2].grid(row = 7, column = 3, pady = (5, 0), sticky = "W")
    increaseFrequencyValve[3] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "+ 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseFrequencyValve[3], frequencyInputValve[3]))
    increaseFrequencyValve[3].grid(row = 12, column = 3, pady = (5, 0), sticky = "W")
    increaseFrequencyValve[4] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "+ 0.2", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseFrequencyValve[4], frequencyInputValve[4]))
    increaseFrequencyValve[4].grid(row = 17, column = 3, pady = (5, 0), sticky = "W")

    # labels automatic control for unique duty cycle 
    dutyCycleValve1Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Duty Cycle :", font = ('Courier New', 14, 'bold'))
    dutyCycleValve1Label.grid(row = 3, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")
    dutyCycleValve2Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Duty Cycle :", font = ('Courier New', 14, 'bold'))
    dutyCycleValve2Label.grid(row = 8, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")
    dutyCycleValve3Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Duty Cycle :", font = ('Courier New', 14, 'bold'))
    dutyCycleValve3Label.grid(row = 13, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")
    dutyCycleValve4Label = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "Duty Cycle :", font = ('Courier New', 14, 'bold'))
    dutyCycleValve4Label.grid(row = 18, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")

    # entry widget for unique duty cycle for each valve
    dutyCycleInputValve[1] = customtkinter.CTkEntry(main_frame["setUniqueInputFrame"], width = 70, font = ('SF Compact', 15, 'bold'), justify = "center")
    dutyCycleInputValve[1].grid(row = 3, column = 1, columnspan = 3, padx = (15, 0), pady = (5, 0), sticky = "W")
    dutyCycleInputValve[1].insert(0, str(int(currentSetDutyCycle1)))
    dutyCycleInputValve[2] = customtkinter.CTkEntry(main_frame["setUniqueInputFrame"], width = 70, font = ('SF Compact', 15, 'bold'), justify = "center")
    dutyCycleInputValve[2].grid(row = 8, column = 1, columnspan = 3, padx = (15, 0), pady = (5, 0), sticky = "W")
    dutyCycleInputValve[2].insert(0, str(int(currentSetDutyCycle2)))
    dutyCycleInputValve[3] = customtkinter.CTkEntry(main_frame["setUniqueInputFrame"], width = 70, font = ('SF Compact', 15, 'bold'), justify = "center")
    dutyCycleInputValve[3].grid(row = 13, column = 1, columnspan = 3, padx = (15, 0), pady = (5, 0), sticky = "W")
    dutyCycleInputValve[3].insert(0, str(int(currentSetDutyCycle3)))
    dutyCycleInputValve[4] = customtkinter.CTkEntry(main_frame["setUniqueInputFrame"], width = 70, font = ('SF Compact', 15, 'bold'), justify = "center")
    dutyCycleInputValve[4].grid(row = 18, column = 1, columnspan = 3, padx = (15, 0), pady = (5, 0), sticky = "W")
    dutyCycleInputValve[4].insert(0, str(int(currentSetDutyCycle4)))
    dutyCycleValve1Unit = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "%", font = ('Courier New', 14, 'bold'))
    dutyCycleValve1Unit.grid(row = 3, column = 2, padx = (50, 0), pady = (5, 0), sticky = "W")
    dutyCycleValve2Unit = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "%", font = ('Courier New', 14, 'bold'))
    dutyCycleValve2Unit.grid(row = 8, column = 2, padx = (50, 0), pady = (5, 0), sticky = "W")
    dutyCycleValve3Unit = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "%", font = ('Courier New', 14, 'bold'))
    dutyCycleValve3Unit.grid(row = 13, column = 2, padx = (50, 0), pady = (5, 0), sticky = "W")
    dutyCycleValve4Unit = customtkinter.CTkLabel(main_frame["setUniqueInputFrame"], text = "%", font = ('Courier New', 14, 'bold'))
    dutyCycleValve4Unit.grid(row = 18, column = 2, padx = (50, 0), pady = (5, 0), sticky = "W")

    # unique duty cycle sliding bar for each valve
    dutyCycleSliderValve[1] = customtkinter.CTkSlider(main_frame["setUniqueInputFrame"], width = 100, from_ = 0, to = 100, number_of_steps = 10, command = lambda value: uniqueDutyCycleSlider(value, 1))
    dutyCycleSliderValve[1].grid(row = 4, column = 2, padx = 10, pady = 10, sticky = "W")
    dutyCycleSliderValve[1].set(int(currentSetDutyCycle1))
    dutyCycleSliderValve[2] = customtkinter.CTkSlider(main_frame["setUniqueInputFrame"], width = 100, from_ = 0, to = 100, number_of_steps = 10, command = lambda value: uniqueDutyCycleSlider(value, 2))
    dutyCycleSliderValve[2].grid(row = 9, column = 2, padx = 10, pady = 10, sticky = "W")
    dutyCycleSliderValve[2].set(int(currentSetDutyCycle2))
    dutyCycleSliderValve[3] = customtkinter.CTkSlider(main_frame["setUniqueInputFrame"], width = 100, from_ = 0, to = 100, number_of_steps = 10, command = lambda value: uniqueDutyCycleSlider(value, 3))
    dutyCycleSliderValve[3].grid(row = 14, column = 2, padx = 10, pady = 10, sticky = "W")
    dutyCycleSliderValve[3].set(int(currentSetDutyCycle3))
    dutyCycleSliderValve[4] = customtkinter.CTkSlider(main_frame["setUniqueInputFrame"], width = 100, from_ = 0, to = 100, number_of_steps = 10, command = lambda value: uniqueDutyCycleSlider(value, 4))
    dutyCycleSliderValve[4].grid(row = 19, column = 2, padx = 10, pady = 10, sticky = "W")
    dutyCycleSliderValve[4].set(int(currentSetDutyCycle4))

    # decrement duty cycle button for each valve
    decreaseDutyCycleValve[1] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "- 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseDutyCycleValve[1], dutyCycleInputValve[1]))
    decreaseDutyCycleValve[1].grid(row = 4, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")
    decreaseDutyCycleValve[2] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "- 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseDutyCycleValve[2], dutyCycleInputValve[2]))
    decreaseDutyCycleValve[2].grid(row = 9, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")
    decreaseDutyCycleValve[3] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "- 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseDutyCycleValve[3], dutyCycleInputValve[3]))
    decreaseDutyCycleValve[3].grid(row = 14, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")
    decreaseDutyCycleValve[4] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "- 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementControlMethodButton(decreaseDutyCycleValve[4], dutyCycleInputValve[4]))
    decreaseDutyCycleValve[4].grid(row = 19, column = 1, padx = (15, 0), pady = (5, 0), sticky = "W")

    # increment duty cycle button for each valve
    increaseDutyCycleValve[1] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "+ 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseDutyCycleValve[1], dutyCycleInputValve[1]))
    increaseDutyCycleValve[1].grid(row = 4, column = 3, pady = (5, 0), sticky = "W")
    increaseDutyCycleValve[2] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "+ 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseDutyCycleValve[2], dutyCycleInputValve[2]))
    increaseDutyCycleValve[2].grid(row = 9, column = 3, pady = (5, 0), sticky = "W")
    increaseDutyCycleValve[3] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "+ 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseDutyCycleValve[3], dutyCycleInputValve[3]))
    increaseDutyCycleValve[3].grid(row = 14, column = 3, pady = (5, 0), sticky = "W")
    increaseDutyCycleValve[4] = customtkinter.CTkButton(main_frame["setUniqueInputFrame"], text = "+ 10", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementControlMethodButton(increaseDutyCycleValve[4], dutyCycleInputValve[4]))
    increaseDutyCycleValve[4].grid(row = 19, column = 3, pady = (5, 0), sticky = "W")

    # change gui entire gui background color and font color when switching theme (dark/white)
    if lightDarkMode.get() == "dark":
        main_frame["controlMethod"].configure(fg_color = "#202020")
        main_frame["manual_frame"].configure(fg_color = "#2F2F2F")
        main_frame["auto_frame"].configure(fg_color = "#2F2F2F")
        main_frame["setUniqueInputFrame"].configure(fg_color = "#3F3F3F")
        main_frame["setSimilarInputFrame"].configure(fg_color = "#3F3F3F")
        manualControlLabel.configure(fg_color = "#2F2F2F", text_color = "white")
        autoControlLabel.configure(fg_color = "#2F2F2F", text_color = "white")
        if manualValveControl[1].get() == 0:
            manualValveControl[1].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
        elif manualValveControl[1].get() == 1:
            manualValveControl[1].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        if manualValveControl[2].get() == 0:
            manualValveControl[2].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
        elif manualValveControl[2].get() == 1:
            manualValveControl[2].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        if manualValveControl[3].get() == 0:
            manualValveControl[3].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
        elif manualValveControl[3].get() == 1:
            manualValveControl[3].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        if manualValveControl[4].get() == 0:
            manualValveControl[4].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
        elif manualValveControl[4].get() == 1:
            manualValveControl[4].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        similarFrequencyLabel.configure(fg_color = "#3F3F3F", text_color = "white")
        similarFrequencyInput.configure(fg_color = "#3F3F3F", text_color = "white")
        similarFrequencyUnit.configure(fg_color = "#3F3F3F", text_color = "white")
        frequency_slider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        decreaseSimilarFrequency.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseSimilarFrequency.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        similarDutyCycleLabel.configure(fg_color = "#3F3F3F", text_color = "white")
        similarDutyCycleInput.configure(fg_color = "#3F3F3F", text_color = "white")
        similarDutyCycleUnit.configure(fg_color = "#3F3F3F", text_color = "white")
        dutycycle_slider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        decreaseSimilarDutyCycle.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseSimilarDutyCycle.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        valve1Label.configure(fg_color = "#3F3F3F", text_color = "white")
        valve2Label.configure(fg_color = "#3F3F3F", text_color = "white")
        valve3Label.configure(fg_color = "#3F3F3F", text_color = "white")
        valve4Label.configure(fg_color = "#3F3F3F", text_color = "white")
        frequency1Label.configure(fg_color = "#3F3F3F", text_color = "white")
        frequency2Label.configure(fg_color = "#3F3F3F", text_color = "white")
        frequency3Label.configure(fg_color = "#3F3F3F", text_color = "white")
        frequency4Label.configure(fg_color = "#3F3F3F", text_color = "white")
        frequencyInputValve[1].configure(fg_color = "#3F3F3F", text_color = "white")
        frequencyInputValve[2].configure(fg_color = "#3F3F3F", text_color = "white")
        frequencyInputValve[3].configure(fg_color = "#3F3F3F", text_color = "white")
        frequencyInputValve[4].configure(fg_color = "#3F3F3F", text_color = "white")
        frequency1Unit.configure(fg_color = "#3F3F3F", text_color = "white")
        frequency2Unit.configure(fg_color = "#3F3F3F", text_color = "white")
        frequency3Unit.configure(fg_color = "#3F3F3F", text_color = "white")
        frequency4Unit.configure(fg_color = "#3F3F3F", text_color = "white")
        frequencySliderValve[1].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        frequencySliderValve[2].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        frequencySliderValve[3].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        frequencySliderValve[4].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        decreaseFrequencyValve[1].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        decreaseFrequencyValve[2].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        decreaseFrequencyValve[3].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        decreaseFrequencyValve[4].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseFrequencyValve[1].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseFrequencyValve[2].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseFrequencyValve[3].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseFrequencyValve[4].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        dutyCycleValve1Label.configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleValve2Label.configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleValve3Label.configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleValve4Label.configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleInputValve[1].configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleInputValve[2].configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleInputValve[3].configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleInputValve[4].configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleValve1Unit.configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleValve2Unit.configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleValve3Unit.configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleValve4Unit.configure(fg_color = "#3F3F3F", text_color = "white")
        dutyCycleSliderValve[1].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        dutyCycleSliderValve[2].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        dutyCycleSliderValve[3].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        dutyCycleSliderValve[4].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        decreaseDutyCycleValve[1].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        decreaseDutyCycleValve[2].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        decreaseDutyCycleValve[3].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        decreaseDutyCycleValve[4].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseDutyCycleValve[1].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseDutyCycleValve[2].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseDutyCycleValve[3].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseDutyCycleValve[4].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
    elif lightDarkMode.get() == "light":
        main_frame["controlMethod"].configure(fg_color = "#E6EE9C")
        main_frame["manual_frame"].configure(fg_color = "#F0F4C3")
        main_frame["auto_frame"].configure(fg_color = "#F0F4C3")
        main_frame["setUniqueInputFrame"].configure(fg_color = "#F9FBE7")
        main_frame["setSimilarInputFrame"].configure(fg_color = "#F9FBE7")
        manualControlLabel.configure(fg_color = "#F0F4C3", text_color = "black")
        autoControlLabel.configure(fg_color = "#F0F4C3", text_color = "black")
        if manualValveControl[1].get() == 0:
            manualValveControl[1].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9",text_color = "black")
        elif manualValveControl[1].get() == 1:
            manualValveControl[1].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
        if manualValveControl[2].get() == 0:
            manualValveControl[2].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
        elif manualValveControl[2].get() == 1:
            manualValveControl[2].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
        if manualValveControl[3].get() == 0:
            manualValveControl[3].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
        elif manualValveControl[3].get() == 1:
            manualValveControl[3].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
        if manualValveControl[4].get() == 0:
            manualValveControl[4].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
        elif manualValveControl[4].get() == 1:
            manualValveControl[4].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
        similarFrequencyLabel.configure(fg_color = "#F9FBE7", text_color = "black")
        similarFrequencyInput.configure(fg_color = "#F9FBE7", text_color = "black")
        similarFrequencyUnit.configure(fg_color = "#F9FBE7", text_color = "black")
        frequency_slider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        decreaseSimilarFrequency.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseSimilarFrequency.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        similarDutyCycleLabel.configure(fg_color = "#F9FBE7", text_color = "black")
        similarDutyCycleInput.configure(fg_color = "#F9FBE7", text_color = "black")
        similarDutyCycleUnit.configure(fg_color = "#F9FBE7", text_color = "black")
        dutycycle_slider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        decreaseSimilarDutyCycle.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseSimilarDutyCycle.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        valve1Label.configure(fg_color = "#F9FBE7", text_color = "black")
        valve2Label.configure(fg_color = "#F9FBE7", text_color = "black")
        valve3Label.configure(fg_color = "#F9FBE7", text_color = "black")
        valve4Label.configure(fg_color = "#F9FBE7", text_color = "black")
        frequency1Label.configure(fg_color = "#F9FBE7", text_color = "black")
        frequency2Label.configure(fg_color = "#F9FBE7", text_color = "black")
        frequency3Label.configure(fg_color = "#F9FBE7", text_color = "black")
        frequency4Label.configure(fg_color = "#F9FBE7", text_color = "black")
        frequencyInputValve[1].configure(fg_color = "#F9FBE7", text_color = "black")
        frequencyInputValve[2].configure(fg_color = "#F9FBE7", text_color = "black")
        frequencyInputValve[3].configure(fg_color = "#F9FBE7", text_color = "black")
        frequencyInputValve[4].configure(fg_color = "#F9FBE7", text_color = "black")
        frequency1Unit.configure(fg_color = "#F9FBE7", text_color = "black")
        frequency2Unit.configure(fg_color = "#F9FBE7", text_color = "black")
        frequency3Unit.configure(fg_color = "#F9FBE7", text_color = "black")
        frequency4Unit.configure(fg_color = "#F9FBE7", text_color = "black")
        frequencySliderValve[1].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        frequencySliderValve[2].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        frequencySliderValve[3].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        frequencySliderValve[4].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        decreaseFrequencyValve[1].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        decreaseFrequencyValve[2].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        decreaseFrequencyValve[3].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        decreaseFrequencyValve[4].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseFrequencyValve[1].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseFrequencyValve[2].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseFrequencyValve[3].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseFrequencyValve[4].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        dutyCycleValve1Label.configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleValve2Label.configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleValve3Label.configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleValve4Label.configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleInputValve[1].configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleInputValve[2].configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleInputValve[3].configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleInputValve[4].configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleValve1Unit.configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleValve2Unit.configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleValve3Unit.configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleValve4Unit.configure(fg_color = "#F9FBE7", text_color = "black")
        dutyCycleSliderValve[1].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        dutyCycleSliderValve[2].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        dutyCycleSliderValve[3].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        dutyCycleSliderValve[4].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        decreaseDutyCycleValve[1].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        decreaseDutyCycleValve[2].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        decreaseDutyCycleValve[3].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        decreaseDutyCycleValve[4].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseDutyCycleValve[1].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseDutyCycleValve[2].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseDutyCycleValve[3].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseDutyCycleValve[4].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
def timeSlider(value):
    global currentSetTimeInterval
    global timeIntervalInput
    global timeIntervalSlider
    # Update the current time interval value based on the slider value
    currentSetTimeInterval = timeIntervalSlider.get()

    # Clear the current content in the time interval input field and update it with the new value
    timeIntervalInput.delete(0, END)
    timeIntervalInput.insert(0, str(int(currentSetTimeInterval)))

def incrementTimeIntervalButton(button, entry):
    global currentSetTimeInterval
    global timeIntervalSlider
    # Check if the button is the increase button
    if button == increaseTimeInterval:

        # Check if the current time interval value is less than 60000
        if int(entry.get()) < 60000:
            current = int(entry.get())

            # Increase the time interval by 100
            entry.delete(0, END)
            entry.insert(0, str(current + 100))
            currentSetTimeInterval = entry.get()

            # Set the slider value to the new time interval
            timeIntervalSlider.set(int(currentSetTimeInterval))

def decrementTimeIntervalButton(button, entry):
    global currentSetTimeInterval
    global timeIntervalSlider
    # Check if the button is the decrease button
    if button == decreaseTimeInterval:

        # Check if the current time interval value is greater than 0
        if int(entry.get()) > 0:
            current = int(entry.get())

            # Decrease the time interval by 100
            entry.delete(0, END)
            entry.insert(0, str(current - 100))
            currentSetTimeInterval = entry.get()

            # Set the slider value to the new time interval
            timeIntervalSlider.set(int(currentSetTimeInterval))

def combobox_selection(event):
    global selected_sequence
    global selectSequenceOrderInput
    # Get the selected sequencing order
    selected_sequence = selectSequenceOrderInput.get()
    print(selected_sequence)

def valveSequence_page():
    # create new main frame
    global main_frame
    global sequenceOrderOptions
    global pickValve_status
    global selectSequenceOrderInput
    global sequencingOrderLabel
    global currentSetTimeInterval
    global decreaseTimeInterval
    global increaseTimeInterval
    global timeIntervalInput
    global timeIntervalSlider
    global timeIntervalLabel
    global timeIntervalInput
    global timeIntervalUnit
    global lightDarkMode
    global selected_sequence
    # Check if sequence order options is empty
    if sequenceOrderOptions == []:
        sequenceOrderOptions = [""]
    
    # Create main frame for valve sequencing
    main_frame["valveSequence"] = customtkinter.CTkFrame(root, width = 450, height = 480)
    main_frame["valveSequence"].grid(row = 0, column = 1, padx = 5, rowspan = 4, sticky = "NSWE")
    main_frame["valveSequence"].grid_propagate(0)

    # Create frame for selecting sequencing order
    main_frame["selectSequencingFrame"] = customtkinter.CTkFrame(main_frame["valveSequence"], width = 440, height = 100)
    main_frame["selectSequencingFrame"].grid(row = 0, column = 0, padx = 5, pady = (10, 0), sticky = "NSWE")
    main_frame["selectSequencingFrame"].grid_propagate(0)

    # Create label for sequencing order
    sequencingOrderLabel = customtkinter.CTkLabel(main_frame["selectSequencingFrame"], text = "Valve Sequencing Order : ", font = ('Courier New', 14, 'bold'))
    sequencingOrderLabel.grid(row = 0, column = 0, pady = 15, padx = (15, 0), sticky = "W")

    # Create combobox for selecting sequencing order
    selectSequenceOrderInput = customtkinter.CTkComboBox(main_frame["selectSequencingFrame"], values = sequenceOrderOptions, height = 30, command = combobox_selection)
    selectSequenceOrderInput.grid(row = 0, column = 1, sticky = "W")
    selectSequenceOrderInput.set(selected_sequence)

    # Create frame for time interval
    main_frame["timeIntervalFrame"] = customtkinter.CTkFrame(main_frame["valveSequence"], width = 440, height = 150)
    main_frame["timeIntervalFrame"].grid(row = 1, column = 0, padx = 5, pady = (10, 0), sticky = "NSWE")
    main_frame["timeIntervalFrame"].grid_propagate(0)

    # Create label for time interval
    timeIntervalLabel = customtkinter.CTkLabel(main_frame["timeIntervalFrame"], text = "Time Interval : ", font = ('Courier New', 14, 'bold'))
    timeIntervalLabel.grid(row = 0, column = 0, pady = 15, padx = (15, 0), sticky = "W")

    # Create entry for time interval
    timeIntervalInput = customtkinter.CTkEntry(main_frame["timeIntervalFrame"], width = 70, font = ('SF Compact', 15, 'bold'), justify = "center")
    timeIntervalInput.grid(row = 0, column = 1, columnspan = 3, padx = (10, 0), pady = 15, sticky = "W")
    timeIntervalInput.insert(0, int(currentSetTimeInterval))

    # Create label for time interval unit
    timeIntervalUnit = customtkinter.CTkLabel(main_frame["timeIntervalFrame"], text = "ms", font = ('Courier New', 14, 'bold'))
    timeIntervalUnit.grid(row = 0, column = 2, padx = (50, 0), pady = 15, sticky = "W")

    # Create slider for time interval
    timeIntervalSlider = customtkinter.CTkSlider(main_frame["timeIntervalFrame"], width = 100, from_ = 0, to = 60000, number_of_steps = 600, command = timeSlider)
    timeIntervalSlider.grid(row = 1, column = 2, padx = 10, pady = 10, sticky = "W")
    timeIntervalSlider.set(int(currentSetTimeInterval))

    # Create button to decrement time interval
    decreaseTimeInterval = customtkinter.CTkButton(main_frame["timeIntervalFrame"], text = "- 100", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: decrementTimeIntervalButton(decreaseTimeInterval, timeIntervalInput))
    decreaseTimeInterval.grid(row = 1, column = 1, padx = (10, 0), pady = (5, 0), sticky = "W")

    # Create button to increment time interval
    increaseTimeInterval = customtkinter.CTkButton(main_frame["timeIntervalFrame"], text = "+ 100", font = ('SF Compact', 15, 'bold'), width = 5, command = lambda: incrementTimeIntervalButton(increaseTimeInterval, timeIntervalInput))
    increaseTimeInterval.grid(row = 1, column = 3, sticky = "W")

    # change gui entire gui background color and font color when switching theme (dark/white)
    if lightDarkMode.get() == "dark":
        main_frame["valveSequence"].configure(fg_color = "#202020")
        main_frame["selectSequencingFrame"].configure(fg_color = "#2f2f2f")
        main_frame["timeIntervalFrame"].configure(fg_color = "#2f2f2f") 
        sequencingOrderLabel.configure(fg_color = "#2f2f2f", text_color = "white")
        selectSequenceOrderInput.configure(fg_color = "#2f2f2f", text_color = "white", button_color = "#4169E1", button_hover_color = "#737CA1")
        timeIntervalLabel.configure(fg_color = "#2f2f2f", text_color = "white")
        timeIntervalInput.configure(fg_color = "#2f2f2f", text_color = "white")
        timeIntervalUnit.configure(fg_color = "#2f2f2f", text_color = "white")
        timeIntervalSlider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        decreaseTimeInterval.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseTimeInterval.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
    if lightDarkMode.get() == "light":
        main_frame["valveSequence"].configure(fg_color = "#F0F4C3")
        main_frame["selectSequencingFrame"].configure(fg_color = "#F9FBE7")
        main_frame["timeIntervalFrame"].configure(fg_color = "#F9FBE7") 
        sequencingOrderLabel.configure(fg_color = "#F9FBE7", text_color = "black")
        selectSequenceOrderInput.configure(fg_color = "#F9FBE7", text_color = "black", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        timeIntervalLabel.configure(fg_color = "#F9FBE7", text_color = "black")
        timeIntervalInput.configure(fg_color = "#F9FBE7", text_color = "black")
        timeIntervalUnit.configure(fg_color = "#F9FBE7", text_color = "black")
        timeIntervalSlider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        decreaseTimeInterval.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseTimeInterval.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")

def toggle_switch(switch):
    global lightDarkMode
    global sequenceOrderOptions
    global pin_val
    global manualValveControl
    global pickValve_status
    global valveTurnOn
    global main_frame
    # Initialize valveTurnOn and sequenceOrderOptions lists
    valveTurnOn = []
    sequenceOrderOptions = []

    # Check the state of the switch
    if switch.get() == 1:

        # Iterate over pickValve_status dictionary
        for pin_index, key in enumerate(pickValve_status):

            # Check if the valve is selected and manual control is ON
            if pickValve_status[key].get() == 1:
                if manualValveControl.get(key) and manualValveControl[key].get() == 1:
                    GPIO.output(pin_val[pin_index], GPIO.HIGH)
                else:
                    GPIO.output(pin_val[pin_index], GPIO.LOW)

            # Append valve key to valveTurnOn list if it's selected
            if pickValve_status[key].get() == 1:
                valveTurnOn.append(key)

        # Configure switch colors based on light/dark mode
        if lightDarkMode.get() == "dark":
            switch.configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1")
        elif lightDarkMode.get() == "light":
            switch.configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9")
    elif switch.get() == 0:
    
        # Iterate over pickValve_status dictionary
        for pin_index, key in enumerate(pickValve_status):

            # Turn OFF the valve if it's not selected or manual control is OFF
            if pickValve_status[key].get() == 0:
                if manualValveControl.get(key) and manualValveControl[key].get() == 1:
                    GPIO.output(pin_val[pin_index], GPIO.LOW)

            # Append valve key to valveTurnOn list if it's selected
            if pickValve_status[key].get() == 1:
                valveTurnOn.append(key)

        # Configure switch colors based on light/dark mode
        if lightDarkMode.get() == "dark":
            switch.configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1")
        elif lightDarkMode.get() == "light":
            switch.configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9")

    # Generate all possible sequencing permutations
    perms = permutations(valveTurnOn)
    for perm in perms:
        sequenceOrderOptions.append(''.join(map(str, perm)))

    # If valve sequence frame exists, destroy and recreate it
    if main_frame.get("valveSequence") and main_frame["valveSequence"].winfo_exists():
        main_frame["valveSequence"].destroy()
        main_frame["selectSequencingFrame"].destroy()
        main_frame["timeIntervalFrame"].destroy()
        indicate(valveSequencing, valveSequence_page)

def changeColorTheme():
    global lightDarkMode
    global main_frame
    global pickValve_status
    global startSavingCSV
    global stopSavingCSV
    global setPressure
    global pressureInput
    global pressureUnit
    global setMotorSpeed
    global motorSpeedInput
    global motorSpeedUnit
    global similarFrequencyLabel
    global similarFrequencyInput
    global similarFrequencyUnit
    global similarDutyCycleLabel
    global similarDutyCycleInput
    global similarDutyCycleUnit
    global manualControlLabel
    global autoControlLabel
    global manualValveControl
    global valve1Label
    global valve2Label
    global valve3Label
    global valve4Label
    global frequency1Label
    global frequency2Label
    global frequency3Label
    global frequency4Label
    global frequency1Unit
    global frequency2Unit
    global frequency3Unit
    global frequency4Unit
    global dutyCycleValve1Label
    global dutyCycleValve2Label
    global dutyCycleValve3Label
    global dutyCycleValve4Label
    global dutyCycleValve1Unit
    global dutyCycleValve2Unit
    global dutyCycleValve3Unit
    global dutyCycleValve4Unit
    global selectSequenceOrderInput
    global sequencingOrderLabel
    global timeIntervalLabel
    global timeIntervalInput
    global timeIntervalUnit
    global timeIntervalSlider
    global decreaseTimeInterval
    global increaseTimeInterval
    global currentButton_page

    # if light theme activated, change all background and font to light theme
    if lightDarkMode.get() == "light":
        customtkinter.set_appearance_mode("light")
        menu_frame.configure(fg_color = "#F0F4C3")
        runTimeDuration_frame.configure(fg_color = "#F0F4C3")
        runTimeDurationLabel.configure(fg_color = "#F0F4C3", text_color = "black")
        runTimeDurationInput.configure(fg_color = "#F0F4C3", text_color = "black")
        runTimeDurationSlider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        decreaseRunTimeDuration.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        increaseRunTimeDuration.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        livaData_frame.configure(fg_color = "#F0F4C3")
        saveData_frame.configure(fg_color = "#F0F4C3")
        runtime_frame.configure(fg_color = "#F0F4C3")
        system_control.configure(fg_color = "#F0F4C3", text_color = "black")
        settings_frame.configure(fg_color = "#F0F4C3")
        valveSelection_frame.configure(fg_color = "#F0F4C3")
        software_title.configure(fg_color = "#F0F4C3", text_color = "black")
        pressure_menu.configure(fg_color = "#F0F4C3", hover_color = "#E8F5E9", text_color = "black")
        controlMode.configure(fg_color = "#F0F4C3", hover_color = "#E8F5E9", text_color = "black")
        valveSequencing.configure(fg_color = "#F0F4C3", hover_color = "#E8F5E9", text_color = "black")
        liveDataLabel.configure(fg_color = "#F0F4C3", text_color = "black")
        startSavingCSV.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        stopSavingCSV.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        pickValveLabel.configure(fg_color = "#F0F4C3", text_color = "black")
        settingsLabel.configure(fg_color = "#F0F4C3", text_color = "black")
        saveData_Label.configure(fg_color = "#F0F4C3", text_color = "black")
        start_Button.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        stop_Button.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        dutyCycleLiveDataButton.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        pressureLiveDataButton.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        changeThemeSwitch.configure(progress_color = "#4CAF50", button_color = "#2E7D32", button_hover_color = "#E8F5E9", text_color = "black")
        if valveSequencingModeSwitch.get() == 0:
            valveSequencingModeSwitch.configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
        elif valveSequencingModeSwitch.get() == 1:
            valveSequencingModeSwitch.configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
        if pickValve_status[1].get() == 0:
            pickValve_status[1].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
        elif pickValve_status[1].get() == 1:
            pickValve_status[1].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
        if pickValve_status[2].get() == 0:
            pickValve_status[2].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
        elif pickValve_status[2].get() == 1:
            pickValve_status[2].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
        if pickValve_status[3].get() == 0:
            pickValve_status[3].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
        elif pickValve_status[3].get() == 1:
            pickValve_status[3].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
        if pickValve_status[4].get() == 0:
            pickValve_status[4].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
        elif pickValve_status[4].get() == 1:
            pickValve_status[4].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
        hide_indicator()
        currentButton_page.configure(fg_color = "#A5D6A7")
        if main_frame.get("fluidControl_page") and main_frame["fluidControl_page"].winfo_exists():
            main_frame["fluidControl_page"].configure(fg_color = "#F0F4C3")
            main_frame["pressure"].configure(fg_color = "#F9FBE7")
            main_frame["motorSpeed"].configure(fg_color = "#F9FBE7")
            setPressure.configure(fg_color = "#F9FBE7",text_color = "black")
            pressureInput.configure(fg_color = "#F9FBE7", text_color = "black")
            pressureUnit.configure(fg_color = "#F9FBE7", text_color = "black")
            decreasePressure.configure(fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9")
            increasePressure.configure(fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9")
            pressure_slider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            setMotorSpeed.configure(fg_color = "#F9FBE7", text_color = "black")
            motorSpeedInput.configure(fg_color = "#F9FBE7", text_color = "black")
            motorSpeedUnit.configure(fg_color = "#F9FBE7", text_color = "black")
            decreaseSetMotorSpeed.configure(fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9")
            increaseSetMotorSpeed.configure(fg_color = "#A5D6A7", text_color = "black", hover_color = "#E8F5E9")
            micropump_slider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
        if main_frame.get("controlMethod") and main_frame["controlMethod"].winfo_exists():
            main_frame["controlMethod"].configure(fg_color = "#E6EE9C")
            main_frame["manual_frame"].configure(fg_color = "#F0F4C3")
            main_frame["auto_frame"].configure(fg_color = "#F0F4C3")
            main_frame["setUniqueInputFrame"].configure(fg_color = "#F9FBE7")
            main_frame["setSimilarInputFrame"].configure(fg_color = "#F9FBE7")
            manualControlLabel.configure(fg_color = "#F0F4C3", text_color = "black")
            autoControlLabel.configure(fg_color = "#F0F4C3", text_color = "black")
            if manualValveControl[1].get() == 0:
                manualValveControl[1].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9",text_color = "black")
            elif manualValveControl[1].get() == 1:
                manualValveControl[1].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
            if manualValveControl[2].get() == 0:
                manualValveControl[2].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
            elif manualValveControl[2].get() == 1:
                manualValveControl[2].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
            if manualValveControl[3].get() == 0:
                manualValveControl[3].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
            elif manualValveControl[3].get() == 1:
                manualValveControl[3].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
            if manualValveControl[4].get() == 0:
                manualValveControl[4].configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9", text_color = "black")
            elif manualValveControl[4].get() == 1:
                manualValveControl[4].configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9", text_color = "black")
            similarFrequencyLabel.configure(fg_color = "#F9FBE7", text_color = "black")
            similarFrequencyInput.configure(fg_color = "#F9FBE7", text_color = "black")
            similarFrequencyUnit.configure(fg_color = "#F9FBE7", text_color = "black")
            frequency_slider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            decreaseSimilarFrequency.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseSimilarFrequency.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            similarDutyCycleLabel.configure(fg_color = "#F9FBE7", text_color = "black")
            similarDutyCycleInput.configure(fg_color = "#F9FBE7", text_color = "black")
            similarDutyCycleUnit.configure(fg_color = "#F9FBE7", text_color = "black")
            dutycycle_slider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            decreaseSimilarDutyCycle.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseSimilarDutyCycle.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            valve1Label.configure(fg_color = "#F9FBE7", text_color = "black")
            valve2Label.configure(fg_color = "#F9FBE7", text_color = "black")
            valve3Label.configure(fg_color = "#F9FBE7", text_color = "black")
            valve4Label.configure(fg_color = "#F9FBE7", text_color = "black")
            frequency1Label.configure(fg_color = "#F9FBE7", text_color = "black")
            frequency2Label.configure(fg_color = "#F9FBE7", text_color = "black")
            frequency3Label.configure(fg_color = "#F9FBE7", text_color = "black")
            frequency4Label.configure(fg_color = "#F9FBE7", text_color = "black")
            frequencyInputValve[1].configure(fg_color = "#F9FBE7", text_color = "black")
            frequencyInputValve[2].configure(fg_color = "#F9FBE7", text_color = "black")
            frequencyInputValve[3].configure(fg_color = "#F9FBE7", text_color = "black")
            frequencyInputValve[4].configure(fg_color = "#F9FBE7", text_color = "black")
            frequency1Unit.configure(fg_color = "#F9FBE7", text_color = "black")
            frequency2Unit.configure(fg_color = "#F9FBE7", text_color = "black")
            frequency3Unit.configure(fg_color = "#F9FBE7", text_color = "black")
            frequency4Unit.configure(fg_color = "#F9FBE7", text_color = "black")
            frequencySliderValve[1].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            frequencySliderValve[2].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            frequencySliderValve[3].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            frequencySliderValve[4].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            decreaseFrequencyValve[1].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            decreaseFrequencyValve[2].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            decreaseFrequencyValve[3].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            decreaseFrequencyValve[4].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseFrequencyValve[1].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseFrequencyValve[2].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseFrequencyValve[3].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseFrequencyValve[4].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            dutyCycleValve1Label.configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleValve2Label.configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleValve3Label.configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleValve4Label.configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleInputValve[1].configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleInputValve[2].configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleInputValve[3].configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleInputValve[4].configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleValve1Unit.configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleValve2Unit.configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleValve3Unit.configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleValve4Unit.configure(fg_color = "#F9FBE7", text_color = "black")
            dutyCycleSliderValve[1].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            dutyCycleSliderValve[2].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            dutyCycleSliderValve[3].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            dutyCycleSliderValve[4].configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            decreaseDutyCycleValve[1].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            decreaseDutyCycleValve[2].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            decreaseDutyCycleValve[3].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            decreaseDutyCycleValve[4].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseDutyCycleValve[1].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseDutyCycleValve[2].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseDutyCycleValve[3].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseDutyCycleValve[4].configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
        if main_frame.get("valveSequence") and main_frame["valveSequence"].winfo_exists():
            main_frame["valveSequence"].configure(fg_color = "#F0F4C3")
            main_frame["selectSequencingFrame"].configure(fg_color = "#F9FBE7")
            main_frame["timeIntervalFrame"].configure(fg_color = "#F9FBE7") 
            sequencingOrderLabel.configure(fg_color = "#F9FBE7", text_color = "black")
            selectSequenceOrderInput.configure(fg_color = "#F9FBE7", text_color = "black", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            timeIntervalLabel.configure(fg_color = "#F9FBE7", text_color = "black")
            timeIntervalInput.configure(fg_color = "#F9FBE7", text_color = "black")
            timeIntervalUnit.configure(fg_color = "#F9FBE7", text_color = "black")
            timeIntervalSlider.configure(progress_color = "#66BB6A", fg_color = "#DCE775", button_color = "#A5D6A7", button_hover_color = "#E8F5E9")
            decreaseTimeInterval.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")
            increaseTimeInterval.configure(fg_color = "#A5D6A7", hover_color = "#E8F5E9", text_color = "black")

    # if dark theme activated, change all background and font to dark theme
    else:
        customtkinter.set_appearance_mode("dark")
        menu_frame.configure(fg_color = "#2F2F2F")
        runTimeDuration_frame.configure(fg_color = "#2F2F2F")
        runTimeDurationLabel.configure(fg_color = "#2F2F2F", text_color = "white")
        runTimeDurationInput.configure(fg_color = "#2F2F2F", text_color = "white")
        runTimeDurationSlider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        decreaseRunTimeDuration.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        increaseRunTimeDuration.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        livaData_frame.configure(fg_color = "#2F2F2F")
        runtime_frame.configure(fg_color = "#2F2F2F")
        system_control.configure(fg_color = "#2F2F2F", text_color = "white")
        valveSelection_frame.configure(fg_color = "#2F2F2F")
        settings_frame.configure(fg_color = "#2F2F2F")
        software_title.configure(fg_color = "#2F2F2F", text_color = "white")
        pressure_menu.configure(fg_color = "#2F2F2F", hover_color = "#737CA1",  text_color = "white")
        controlMode.configure(fg_color = "#2F2F2F", hover_color = "#737CA1", text_color = "white")
        liveDataLabel.configure(fg_color = "#2F2F2F", text_color = "white")
        valveSequencing.configure(fg_color = "#2F2F2F", hover_color = "#737CA1", text_color = "white")
        pickValveLabel.configure(fg_color = "#2F2F2F", text_color = "white")
        settingsLabel.configure(fg_color = "#2F2F2F", text_color = "white")
        saveData_Label.configure(fg_color = "#2F2F2F", text_color = "white")
        startSavingCSV.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        stopSavingCSV.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        start_Button.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        stop_Button.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        dutyCycleLiveDataButton.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        pressureLiveDataButton.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        changeThemeSwitch.configure(fg_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        if valveSequencingModeSwitch.get() == 0:
            valveSequencingModeSwitch.configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
        elif valveSequencingModeSwitch.get() == 1:
            valveSequencingModeSwitch.configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        if pickValve_status[1].get() == 0:
            pickValve_status[1].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
        elif pickValve_status[1].get() == 1:
            pickValve_status[1].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        if pickValve_status[2].get() == 0:
            pickValve_status[2].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
        elif pickValve_status[2].get() == 1:
            pickValve_status[2].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        if pickValve_status[3].get() == 0:
            pickValve_status[3].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
        elif pickValve_status[3].get() == 1:
            pickValve_status[3].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        if pickValve_status[4].get() == 0:
            pickValve_status[4].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
        elif pickValve_status[4].get() == 1:
            pickValve_status[4].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
        hide_indicator()
        currentButton_page.configure(fg_color = "#4169E1")
        if main_frame.get("fluidControl_page") and main_frame["fluidControl_page"].winfo_exists():
            main_frame["fluidControl_page"].configure(fg_color = "#2F2F2F")
            main_frame["pressure"].configure(fg_color = "#3F3F3F")
            main_frame["motorSpeed"].configure(fg_color = "#3F3F3F")
            setPressure.configure(fg_color = "#3F3F3F", text_color = "white")
            pressureInput.configure(fg_color = "#3F3F3F", text_color = "white")
            pressureUnit.configure(fg_color = "#3F3F3F", text_color = "white")
            decreasePressure.configure(fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1")
            increasePressure.configure(fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1")
            pressure_slider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            setMotorSpeed.configure(fg_color = "#3F3F3F", text_color = "white")
            motorSpeedInput.configure(fg_color = "#3F3F3F", text_color = "white")
            motorSpeedUnit.configure(fg_color = "#3F3F3F", text_color = "white")
            decreaseSetMotorSpeed.configure(fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1")
            increaseSetMotorSpeed.configure(fg_color = "#4169E1", text_color = "white", hover_color = "#737CA1")
            micropump_slider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
        if main_frame.get("controlMethod") and main_frame["controlMethod"].winfo_exists():
            main_frame["controlMethod"].configure(fg_color = "#202020")
            main_frame["manual_frame"].configure(fg_color = "#2F2F2F")
            main_frame["auto_frame"].configure(fg_color = "#2F2F2F")
            main_frame["setUniqueInputFrame"].configure(fg_color = "#3F3F3F")
            main_frame["setSimilarInputFrame"].configure(fg_color = "#3F3F3F")
            manualControlLabel.configure(fg_color = "#2F2F2F", text_color = "white")
            autoControlLabel.configure(fg_color = "#2F2F2F", text_color = "white")
            if manualValveControl[1].get() == 0:
                manualValveControl[1].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
            elif manualValveControl[1].get() == 1:
                manualValveControl[1].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
            if manualValveControl[2].get() == 0:
                manualValveControl[2].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
            elif manualValveControl[2].get() == 1:
                manualValveControl[2].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
            if manualValveControl[3].get() == 0:
                manualValveControl[3].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
            elif manualValveControl[3].get() == 1:
                manualValveControl[3].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
            if manualValveControl[4].get() == 0:
                manualValveControl[4].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
            elif manualValveControl[4].get() == 1:
                manualValveControl[4].configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
            similarFrequencyLabel.configure(fg_color = "#3F3F3F", text_color = "white")
            similarFrequencyInput.configure(fg_color = "#3F3F3F", text_color = "white")
            similarFrequencyUnit.configure(fg_color = "#3F3F3F", text_color = "white")
            frequency_slider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            decreaseSimilarFrequency.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseSimilarFrequency.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            similarDutyCycleLabel.configure(fg_color = "#3F3F3F", text_color = "white")
            similarDutyCycleInput.configure(fg_color = "#3F3F3F", text_color = "white")
            similarDutyCycleUnit.configure(fg_color = "#3F3F3F", text_color = "white")
            dutycycle_slider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            decreaseSimilarDutyCycle.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseSimilarDutyCycle.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            valve1Label.configure(fg_color = "#3F3F3F", text_color = "white")
            valve2Label.configure(fg_color = "#3F3F3F", text_color = "white")
            valve3Label.configure(fg_color = "#3F3F3F", text_color = "white")
            valve4Label.configure(fg_color = "#3F3F3F", text_color = "white")
            frequency1Label.configure(fg_color = "#3F3F3F", text_color = "white")
            frequency2Label.configure(fg_color = "#3F3F3F", text_color = "white")
            frequency3Label.configure(fg_color = "#3F3F3F", text_color = "white")
            frequency4Label.configure(fg_color = "#3F3F3F", text_color = "white")
            frequencyInputValve[1].configure(fg_color = "#3F3F3F", text_color = "white")
            frequencyInputValve[2].configure(fg_color = "#3F3F3F", text_color = "white")
            frequencyInputValve[3].configure(fg_color = "#3F3F3F", text_color = "white")
            frequencyInputValve[4].configure(fg_color = "#3F3F3F", text_color = "white")
            frequency1Unit.configure(fg_color = "#3F3F3F", text_color = "white")
            frequency2Unit.configure(fg_color = "#3F3F3F", text_color = "white")
            frequency3Unit.configure(fg_color = "#3F3F3F", text_color = "white")
            frequency4Unit.configure(fg_color = "#3F3F3F", text_color = "white")
            frequencySliderValve[1].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            frequencySliderValve[2].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            frequencySliderValve[3].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            frequencySliderValve[4].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            decreaseFrequencyValve[1].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            decreaseFrequencyValve[2].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            decreaseFrequencyValve[3].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            decreaseFrequencyValve[4].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseFrequencyValve[1].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseFrequencyValve[2].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseFrequencyValve[3].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseFrequencyValve[4].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            dutyCycleValve1Label.configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleValve2Label.configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleValve3Label.configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleValve4Label.configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleInputValve[1].configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleInputValve[2].configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleInputValve[3].configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleInputValve[4].configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleValve1Unit.configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleValve2Unit.configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleValve3Unit.configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleValve4Unit.configure(fg_color = "#3F3F3F", text_color = "white")
            dutyCycleSliderValve[1].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            dutyCycleSliderValve[2].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            dutyCycleSliderValve[3].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            dutyCycleSliderValve[4].configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            decreaseDutyCycleValve[1].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            decreaseDutyCycleValve[2].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            decreaseDutyCycleValve[3].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            decreaseDutyCycleValve[4].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseDutyCycleValve[1].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseDutyCycleValve[2].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseDutyCycleValve[3].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseDutyCycleValve[4].configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
        if main_frame.get("valveSequence") and main_frame["valveSequence"].winfo_exists():
            main_frame["valveSequence"].configure(fg_color = "#202020")
            main_frame["selectSequencingFrame"].configure(fg_color = "#2f2f2f")
            main_frame["timeIntervalFrame"].configure(fg_color = "#2f2f2f") 
            sequencingOrderLabel.configure(fg_color = "#2f2f2f", text_color = "white")
            selectSequenceOrderInput.configure(fg_color = "#2f2f2f", text_color = "white", button_color = "#4169E1", button_hover_color = "#737CA1")
            timeIntervalLabel.configure(fg_color = "#2f2f2f", text_color = "white")
            timeIntervalInput.configure(fg_color = "#2f2f2f", text_color = "white")
            timeIntervalUnit.configure(fg_color = "#2f2f2f", text_color = "white")
            timeIntervalSlider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
            decreaseTimeInterval.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
            increaseTimeInterval.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")

def decrementRunTimeDurationButton(button, entry):
    global currentSetRunTimeDuration
    global runTimeDurationInput
    # Initialize current
    current = 0

    # Check if the duration is 1, then set it to infinity
    if int(entry.get()) == 1:
        entry.delete(0, END)
        runTimeDurationInput.insert(0, "∞")

    # If the duration is greater than 1, decrement it
    elif int(entry.get()) > 1:
        current = int(entry.get())
        entry.delete(0, END)
        currentSetRunTimeDuration = int(current - 1)
        runTimeDurationInput.insert(0, str(currentSetRunTimeDuration))
def incrementRunTimeDurationButton(button, entry):
    global currentSetRunTimeDuration
    global runTimeDurationInput
    # Initialize current
    current = 0

    # Check if the duration is infinity, then set it to 1
    if str(entry.get()) == "∞":
        entry.delete(0, END)
        runTimeDurationInput.insert(0, str(1))

    # If the duration is less than 301, increment it
    elif int(entry.get()) < 301:
        current = int(entry.get())
        entry.delete(0, END)
        currentSetRunTimeDuration = int(current + 1)
        runTimeDurationInput.insert(0, str(currentSetRunTimeDuration))
def runTimeDurationSlider(value):
    global currentSetRunTimeDuration
    global runTimeDurationInput
    global runTimeDurationSlider
    # Set current duration
    currentSetRunTimeDuration = runTimeDurationSlider.get()
    runTimeDurationInput.delete(0, END)

    # If the duration is 0, display infinity, else display the duration
    if currentSetRunTimeDuration == 0:
        runTimeDurationInput.insert(0, "∞")
    else:
        runTimeDurationInput.insert(0, str(int(currentSetRunTimeDuration)))
def toggleValveSequenceSwitch():
    global valveSequencingModeSwitch
    global lightDarkMode
    # Check if the switch is on
    if valveSequencingModeSwitch.get() == 1:

        # Change colors based on light/dark mode
        if lightDarkMode.get() == "dark":
            valveSequencingModeSwitch.configure(progress_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1")
        elif lightDarkMode.get() == "light":
            valveSequencingModeSwitch.configure(progress_color = "#B2FF59", button_color = "#00E676", button_hover_color = "#E8F5E9")

    # Check if the switch is off
    elif valveSequencingModeSwitch.get() == 0:

        # Change colors based on light/dark mode
        if lightDarkMode.get() == "dark":
            valveSequencingModeSwitch.configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1")
        elif lightDarkMode.get() == "light":
            valveSequencingModeSwitch.configure(fg_color = "#FFAB91", button_color = "#FF7043", button_hover_color = "#E8F5E9")

def acquireSensorData():
    global sensor_data
    global saveData
    global pin_val
    global pressureReader
    # Initialize sensor_data list
    sensor_data = []

    # Read sensor data while saveData flag is True
    while saveData:

        # Read GPIO pin states
        pin_state1 = GPIO.input(pin_val[0])
        pin_state2 = GPIO.input(pin_val[1])
        pin_state3 = GPIO.input(pin_val[2])
        pin_state4 = GPIO.input(pin_val[3])

        # Get current timestamp
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # Assign pin states and pressure readings        
        valve1state = pin_state1
        valve2state = pin_state2
        valve3state = pin_state3
        valve4state = pin_state4
        pressureOutput1 = pressureReader[0]
        pressureOutput2 = pressureReader[1]
        # Simulate random pressure readings for valves 3 and 4
        pressureOutput3 = random.randint(0, 5)
        pressureOutput4 = random.randint(0, 5)

        # Append data to sensor_data list
        sensor_data.append((timestamp, valve1state, valve2state, valve3state, valve4state, pressureOutput1, pressureOutput2, pressureOutput3, pressureOutput4))
        
        # Wait for 0.02 seconds before acquiring next data
        time.sleep(0.02)

def saveSensorDataToCSV(data, filename):
    # Open CSV file for writing
    with open(filename, "w", newline='') as csvfile:
        file_exists = os.path.isfile(filename)
        csv_writer = csv.writer(csvfile)

        # Write header row
        csv_writer.writerow(['Time', 'Valve 1', 'Valve 2', 'Valve 3', 'Valve 4', 'Pressure Output (Valve 1)', 'Pressure Output (Valve 2)', 'Pressure Output (Valve 3)', 'Pressure Output (Valve 4)'])
        
        # Write data rows
        for timestamp, valve1state, valve2state, valve3state, valve4state, pressureOutput1, pressureOutput2, pressureOutput3, pressureOutput4 in data:
            csv_writer.writerow([timestamp, valve1state, valve2state, valve3state, valve4state, pressureOutput1, pressureOutput2, pressureOutput3, pressureOutput4])
def savingDataIntoCSV():
    global saveData
    global lightDarkMode

    # Start saving data if not already saving
    if saveData == False:

        # Create a notification window (change background color depending on color theme)
        if lightDarkMode.get() == "dark":
            notifyUser = customtkinter.CTkToplevel(root, fg_color = "#b7bac5")
            notifyUser.overrideredirect(True)
            notifyUser.geometry("250x100")
            notifyUser.geometry('+300+200')
            notifyUserLabel = customtkinter.CTkLabel(notifyUser, text = "Start Saving....", font = ('MS Sans Serif', 14, 'bold'), text_color = "white")
            notifyUserLabel.grid(row = 0, column = 0, padx = (10, 0), pady = (15, 0), sticky = "w")
            closeNotifyUserButton = customtkinter.CTkButton(notifyUser, text = "Ok", font = ('MS Sans Serif', 14, 'bold'), width = 30, text_color = "white", fg_color = "#4169E1", hover_color = "#737CA1", corner_radius = 5, anchor = "w", command = lambda: closeNotification(notifyUser))
            closeNotifyUserButton.grid(row = 1, column = 0, padx = (10, 0), pady =  (5, 0), sticky = "w")
        elif lightDarkMode.get() == "light":
            notifyUser = customtkinter.CTkToplevel(root, fg_color = "#ebf6f7")
            notifyUser.overrideredirect(True)
            notifyUser.geometry("250x100")
            notifyUser.geometry('+300+200')
            notifyUserLabel = customtkinter.CTkLabel(notifyUser, text = "Start Saving....", font = ('MS Sans Serif', 14, 'bold'), text_color = "black")
            notifyUserLabel.grid(row = 0, column = 0, padx = (10, 0), pady = (15, 0), sticky = "w")
            closeNotifyUserButton = customtkinter.CTkButton(notifyUser, text = "Ok", font = ('MS Sans Serif', 14, 'bold'), width = 30, text_color = "black",fg_color = "#A5D6A7", hover_color = "#E8F5E9", corner_radius = 5, anchor = "w", command = lambda: closeNotification(notifyUser))
            closeNotifyUserButton.grid(row = 1, column = 0, padx = (10, 0), pady =  (5, 0), sticky = "w")

        # Start saving data thread
        saveData = True
        acquisition_thread = threading.Thread(target=acquireSensorData)
        acquisition_thread.daemon = True
        acquisition_thread.start()
    else:
        pass

def closeNotification(notifyUser):
    # Close notification window
    notifyUser.destroy()

def stopSavingDataIntoCSV():
    global saveData
    global sensor_data
    # Stop saving data if currently saving
    if saveData == True:
        # Create a notification window (change background color depending on color theme)
        if lightDarkMode.get() == "dark":
            stopSavingWindow = customtkinter.CTkToplevel(root, fg_color = "#b7bac5")
            stopSavingWindow.overrideredirect(True)
            stopSavingWindow.geometry("250x100")
            stopSavingWindow.geometry('+300+200')
            enterFileNameLabel = customtkinter.CTkLabel(stopSavingWindow, text = "Save Successful", font = ('MS Sans Serif', 14, 'bold'), text_color = "white")
            enterFileNameLabel.grid(row = 0, column = 0, padx = (10, 0), pady = (15, 0), sticky = "w")
            closeNotifyUserButton = customtkinter.CTkButton(stopSavingWindow, text = "Ok", font = ('MS Sans Serif', 14, 'bold'), width = 30, text_color = "white", fg_color = "#4169E1", hover_color = "#737CA1", corner_radius = 5, anchor = "w", command = lambda: closeNotification(stopSavingWindow))
            closeNotifyUserButton.grid(row = 1, column = 0, padx = (10, 0), pady =  (5, 0), sticky = "w")
        elif lightDarkMode.get() == "light":
            stopSavingWindow = customtkinter.CTkToplevel(root, fg_color = "#ebf6f7")
            stopSavingWindow.overrideredirect(True)
            stopSavingWindow.geometry("250x100")
            stopSavingWindow.geometry('+300+200')
            enterFileNameLabel = customtkinter.CTkLabel(stopSavingWindow, text = "Save Successful", font = ('MS Sans Serif', 14, 'bold'), text_color = "black")
            enterFileNameLabel.grid(row = 0, column = 0, padx = (10, 0), pady = (15, 0), sticky = "w")
            closeNotifyUserButton = customtkinter.CTkButton(stopSavingWindow, text = "Ok", font = ('MS Sans Serif', 14, 'bold'), width = 30, text_color = "black",fg_color = "#A5D6A7", hover_color = "#E8F5E9", corner_radius = 5, anchor = "w", command = lambda: closeNotification(stopSavingWindow))
            closeNotifyUserButton.grid(row = 1, column = 0, padx = (10, 0), pady =  (5, 0), sticky = "w")
        
        # Stop saving data
        saveData = False
        
        # Get a unique filename
        counter = count(-1)
        while True:
            nextCount = next(counter)
            filename = f"soft_robot{nextCount}.csv"
            checkFilePath = os.path.expanduser(f"~/Downloads/{filename}")
            if not os.path.isfile(checkFilePath):
                break

        # Construct the full file path
        file_path = os.path.expanduser("~/Downloads/" + filename)

        # Save sensor data to CSV
        saveSensorDataToCSV(sensor_data, file_path)
    else:
        pass

GPIO.setmode(GPIO.BCM)
# Setting up GPIO pins for output
pinConfig = pin_val + motorSpeedPin
GPIO.setup(pinConfig, GPIO.OUT)
# Initialize GPIO pins
gpio_initialisation()
# Initialize PWM for motor speed control
motorSpeedPWM = GPIO.PWM(motorSpeedPin[0], 400)
# Start a thread for reading pressure sensor data
sensor_thread = threading.Thread(target=sensor_feedback)
sensor_thread.daemon = True  # Daemonize the thread to allow program exit without waiting for it
sensor_thread.start()
# Start creating GUI 
# Create menu frame
menu_frame = customtkinter.CTkFrame(root, width = 170, height = 70, fg_color = "#2F2F2F")
menu_frame.grid(row = 0, column = 0, sticky = "NSWE")
menu_frame.grid_propagate(0)
# title of the software
software_title = customtkinter.CTkLabel(menu_frame, text = "PneuTech", font = ('MS Sans Serif', 20, 'bold'))
software_title.grid(row = 0, column = 0, padx = (15, 0), pady = (5,0), sticky = "W")
# menu options - pressure
pressure_menu = customtkinter.CTkButton(menu_frame, text = "   Fluid Control", font = ('MS Sans Serif', 14, 'bold'), width = 174, corner_radius = 0, anchor = "w", command = lambda: indicate(pressure_menu, fluidControl_page))
pressure_menu.grid(row = 1, column = 0, pady = (10, 0), sticky = "W")
# menu options - valve control mode label
controlMode = customtkinter.CTkButton(menu_frame, text = "   Control Mode", font = ('MS Sans Serif', 14, 'bold'), width = 174, corner_radius = 0, anchor = "w", command = lambda: indicate(controlMode, controlMethod_page))
controlMode.grid(row = 2, column = 0, sticky = "W")
# menu options - valve sequencing
valveSequencing = customtkinter.CTkButton(menu_frame, text = "   Auto Sequence", font = ('MS Sans Serif', 14, 'bold'), width = 174, corner_radius = 0, anchor = "w", command = lambda: indicate(valveSequencing, valveSequence_page))
valveSequencing.grid(row = 3, column = 0, sticky = "W")
# set system running time frame
runTimeDuration_frame = customtkinter.CTkFrame(root, width = 170, height = 35)
runTimeDuration_frame.grid(row = 1, column = 0, pady = (5, 0), sticky = "NSWE")
runTimeDuration_frame.grid_propagate(0) 
runTimeDurationLabel = customtkinter.CTkLabel(runTimeDuration_frame, text = "Duration (second) :", font = ('MS Sans Serif', 14, 'bold'))
runTimeDurationLabel.grid(row = 0, column = 0, columnspan = 3, padx = (5, 0), pady = (5, 0), sticky = "W")
runTimeDurationInput = customtkinter.CTkEntry(runTimeDuration_frame, font = ('SF Compact', 12, 'bold'), justify = "center", width = 50)
runTimeDurationInput.grid(row = 1, column = 1, pady = (5, 0), sticky = "EW")
runTimeDurationInput.insert(0, "∞")
runTimeDurationSlider = customtkinter.CTkSlider(runTimeDuration_frame, width = 80, from_ = 0, to = 300, number_of_steps = 301, command = runTimeDurationSlider)
runTimeDurationSlider.grid(row = 2, column = 1, pady = 10, sticky = "W")
runTimeDurationSlider.set(int(currentSetRunTimeDuration))
# decrement time duration button
decreaseRunTimeDuration = customtkinter.CTkButton(runTimeDuration_frame, text = "- 1", font = ('MS Sans Serif', 14, 'bold'), width = 0.5, command = lambda: decrementRunTimeDurationButton(decreaseRunTimeDuration, runTimeDurationInput))
decreaseRunTimeDuration.grid(row = 2, column = 0, padx = 5, pady = (5, 0), sticky = "W")
# increment time duration button
increaseRunTimeDuration = customtkinter.CTkButton(runTimeDuration_frame, text = "+ 1", font = ('MS Sans Serif', 14, 'bold'), width = 0.5, command = lambda: incrementRunTimeDurationButton(increaseRunTimeDuration, runTimeDurationInput))
increaseRunTimeDuration.grid(row = 2, column = 2, pady = (5, 0), sticky = "W")
# data logging frame
livaData_frame = customtkinter.CTkFrame(root, width = 170, height = 35)
livaData_frame.grid(row = 2, column = 0, pady = (5, 0), sticky = "NSWE")
livaData_frame.grid_propagate(0)
# menu options - data logging
liveDataLabel = customtkinter.CTkLabel(livaData_frame, text = "Live Data :", font = ('MS Sans Serif', 14, 'bold'))
liveDataLabel.grid(row = 0, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")
# create manual switch turning on liveData graph : duty cycle over time, pressure output over time, 
dutyCycleLiveDataButton = customtkinter.CTkButton(livaData_frame, text = "Duty Cycle", font = ('MS Sans Serif', 12, 'bold'), command = liveDataValveStatePlot)
dutyCycleLiveDataButton.grid(row = 1, column = 0, padx = (15, 0), pady = (1, 0), sticky = "W")
pressureLiveDataButton = customtkinter.CTkButton(livaData_frame, text = "Pressure", font = ('MS Sans Serif', 12, 'bold'), command = liveDataPressurePlot)
pressureLiveDataButton.grid(row = 2, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")
# save data frame
saveData_frame = customtkinter.CTkFrame(root, width = 170, height = 35)
saveData_frame.grid(row = 2, column = 2, pady = (5, 0), sticky = "NSWE")
saveData_frame.grid_propagate(0)
# menu options - save data label
saveData_Label = customtkinter.CTkLabel(saveData_frame, text = "Save Data Into CSV:", font = ('MS Sans Serif', 13, 'bold'))
saveData_Label.grid(row = 0, column = 0, padx = (15, 0), pady = (2, 0), sticky = "W")
# menu options - save data in csv format 
startSavingCSV = customtkinter.CTkButton(saveData_frame, text = "Save", font = ('MS Sans Serif', 12, 'bold'), width = 5, command = savingDataIntoCSV)
startSavingCSV.grid(row = 1, column = 0, padx = (15, 0), pady = (2, 0), sticky = "W") 
# menu options - stop data saving
stopSavingCSV = customtkinter.CTkButton(saveData_frame, text = "Stop", font = ('MS Sans Serif', 12, 'bold'), width = 5, command = stopSavingDataIntoCSV)
stopSavingCSV.grid(row = 1, column = 0, padx = (70, 0), pady = (2, 0), sticky = "W") 
# start/stop/pause frame
runtime_frame = customtkinter.CTkFrame(root, width = 170, height = 40)
runtime_frame.grid(row = 3, column = 0, pady = (5, 0), sticky = "NSWE")
runtime_frame.grid_propagate(0)
# menu options - system control label
system_control = customtkinter.CTkLabel(runtime_frame, text = "System Control :", font = ('MS Sans Serif', 14, 'bold'))
system_control.grid(row = 0, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")
# menu options - start 
start_Button = customtkinter.CTkButton(runtime_frame, text = "Start", width = 5, font = ('MS Sans Serif', 13, 'bold'), command = automaticRunHardwareControl)
start_Button.grid(row = 1, column = 0, padx = (20, 0), pady = (10, 0), sticky = "W")
# menu options - stop
stop_Button = customtkinter.CTkButton(runtime_frame, text = "Stop", width = 5, font = ('MS Sans Serif', 13, 'bold'), command = stopHardwareAutomaticControl)
stop_Button.grid(row = 1, column = 0, padx = (80, 0), pady = (10, 0), sticky = "W")
# on / off valve sequencing mode
valveSequencingModeSwitch = customtkinter.CTkSwitch(runtime_frame, text = "Sequence Mode", switch_width = 38, switch_height = 20, font = ('MS Sans Serif', 12, 'bold'), command = toggleValveSequenceSwitch)
valveSequencingModeSwitch.grid(row = 10, column = 0, padx = (15, 0), pady = (10, 0), sticky = "WE") 
# create new main frame
valveSelection_frame = customtkinter.CTkFrame(root, width = 170, height = 50)
valveSelection_frame.grid(row = 0, column = 2, rowspan = 2, sticky = "NS")
valveSelection_frame.grid_propagate(0)
# label to remind user to pick a valve
pickValveLabel =  customtkinter.CTkLabel(valveSelection_frame, text = "Select Valves :", font = ('SF Compact', 15, 'bold'))
pickValveLabel.grid(row = 0, column = 0, padx = (15, 0), pady = (10, 0), sticky = "W")
# valve 1 label and valve 1 on/off button
pickValve_status[1] = customtkinter.CTkSwitch(valveSelection_frame, text = "Valve 1", switch_width = 38, switch_height = 20, font = ('MS Sans Serif', 14, 'bold'), command=lambda: toggle_switch(pickValve_status[1]))
pickValve_status[1].grid(row = 1, column = 0, padx = (15, 0), pady = (10, 0), sticky = "W")
# valve 2 label and valve 2 on/off button
pickValve_status[2] = customtkinter.CTkSwitch(valveSelection_frame, text = "Valve 2", switch_width = 38, switch_height = 20, font = ('MS Sans Serif', 14, 'bold'), command=lambda: toggle_switch(pickValve_status[2]))
pickValve_status[2].grid(row = 2, column = 0, padx = (15, 0), pady = (10, 0), sticky = "W")
# valve 3 label and valve 3 on/off button
pickValve_status[3] = customtkinter.CTkSwitch(valveSelection_frame, text = "Valve 3", switch_width = 38, switch_height = 20, font = ('MS Sans Serif', 14, 'bold'), command=lambda: toggle_switch(pickValve_status[3]))
pickValve_status[3].grid(row = 3, column = 0, padx = (15, 0), pady = (10, 0), sticky = "W")
# valve 4 label and valve 4 on/off button
pickValve_status[4] = customtkinter.CTkSwitch(valveSelection_frame, text = "Valve 4", switch_width = 38, switch_height = 20, font = ('MS Sans Serif', 14, 'bold'), command=lambda: toggle_switch(pickValve_status[4]))
pickValve_status[4].grid(row = 4, column = 0, padx = (15, 0), pady = (10, 0), sticky = "W")
# create new main frame
settings_frame = customtkinter.CTkFrame(root, width = 170, height = 40)
settings_frame.grid(row = 3, column = 2, pady = (5, 0), sticky = "NS")
settings_frame.grid_propagate(0)
# label title of the frame
settingsLabel = customtkinter.CTkLabel(settings_frame, text = "Setting :", font = ('MS Sans Serif', 15, 'bold'))
settingsLabel.grid(row = 0, column = 0, padx = (15, 0), pady = (5, 0), sticky = "W")
# switch to change background theme
changeThemeSwitch = customtkinter.CTkSwitch(settings_frame, text = "Appearance", switch_width = 38, switch_height = 20, variable = lightDarkMode, offvalue = "dark", onvalue = "light", font = ('MS Sans Serif', 12, 'bold'), command = changeColorTheme)
changeThemeSwitch.grid(row = 3, column = 0, padx = (15, 0), pady = (10, 0), sticky = "W")
menu_frame.configure(fg_color = "#2F2F2F")
runTimeDuration_frame.configure(fg_color = "#2F2F2F")
runTimeDurationLabel.configure(fg_color = "#2F2F2F", text_color = "white")
runTimeDurationInput.configure(fg_color = "#2F2F2F", text_color = "white")
runTimeDurationSlider.configure(progress_color = "#4169E1", fg_color = "#4F4F4F", button_color = "#4169E1", button_hover_color = "#737CA1")
decreaseRunTimeDuration.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
increaseRunTimeDuration.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
livaData_frame.configure(fg_color = "#2F2F2F")
saveData_frame.configure(fg_color = "#2F2F2F")
runtime_frame.configure(fg_color = "#2F2F2F")
system_control.configure(fg_color = "#2F2F2F", text_color = "white")
valveSelection_frame.configure(fg_color = "#2F2F2F")
settings_frame.configure(fg_color = "#2F2F2F")
software_title.configure(fg_color = "#2F2F2F", text_color = "white")
pressure_menu.configure(fg_color = "#2F2F2F", hover_color = "#737CA1",  text_color = "white")
controlMode.configure(fg_color = "#2F2F2F", hover_color = "#737CA1", text_color = "white")
liveDataLabel.configure(fg_color = "#2F2F2F", text_color = "white")
valveSequencing.configure(fg_color = "#2F2F2F", hover_color = "#737CA1", text_color = "white")
saveData_Label.configure(fg_color = "#2F2F2F", text_color = "white")
pickValveLabel.configure(fg_color = "#2F2F2F", text_color = "white")
settingsLabel.configure(fg_color = "#2F2F2F", text_color = "white")
startSavingCSV.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
stopSavingCSV.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
start_Button.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
stop_Button.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
changeThemeSwitch.configure(fg_color = "#29465B", button_color = "#4169E1", button_hover_color = "#737CA1", text_color = "white")
dutyCycleLiveDataButton.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
pressureLiveDataButton.configure(fg_color = "#4169E1", hover_color = "#737CA1", text_color = "white")
valveSequencingModeSwitch.configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
pickValve_status[1].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
pickValve_status[2].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
pickValve_status[3].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
pickValve_status[4].configure(fg_color = "#7F7F7F", button_color = "#F7FCFE", button_hover_color = "#737CA1", text_color = "white")
# first page open when starting the software
indicate(pressure_menu,fluidControl_page)
root.mainloop()
