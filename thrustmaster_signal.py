import time
import pygame
import json
import serial

# Install pygame library and pyserial with "pip install pyserial" and "pip install pygame" on the terminal prompt

# Initialize Pygame and the Joystick module
pygame.init()
pygame.joystick.init()

# Check if there are any joysticks connected
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joysticks detected")
    pygame.quit()
    exit()
else:
    print(f"Detected {joystick_count} joystick(s)")

# Initialize the first connected joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Optional: Print information about joystick axes, buttons, and hats
num_axes = joystick.get_numaxes()
print(f"Number of axes: {num_axes}")

num_buttons = joystick.get_numbuttons()
print(f"Number of buttons: {num_buttons}")

num_hats = joystick.get_numhats()
print(f"Number of hats: {num_hats}")

# Attempt to send data to Teensy 4.1. PLEASE TURN OFF/CLOSE THE Serial Monitor on Arduino IDE so it can connects
ser = serial.Serial(port='COM26', baudrate=115200, timeout=1)

try:
    while True:
        pygame.event.pump()  # Process the event queue

        # Loop through each axis and publish its value
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            axis_topic = f"rpi/broadcast/axis{i}"
            axis_print = f"Axis {i}:"
            value_str = str(axis_value)
            print(f"{axis_print} {value_str}")

            # Send data over to teensy via Serial
            ser.write(f"{axis_print} {value_str}\n".encode())

        # Loop through each button and publish its state
        for i in range(joystick.get_numbuttons()):
            button_state = joystick.get_button(i)
            button_topic = f"rpi/broadcast/button{i}"
            button_print = f"Button {i}:"
            state_str = str(button_state)
            #print(f"{button_print}: {state_str}")
            ser.write(f"{button_print} {state_str}\n".encode())

        # Loop through each hat and publish its state
        for i in range(joystick.get_numhats()):
            hat_state = joystick.get_hat(i)
            hat_topic = f"rpi/broadcast/hat{i}"
            hat_print = f"Hat {i}:"
            state_str = str(hat_state)  # hat_state is a tuple (x, y), so we convert it to string
            print(f"{hat_print} {state_str}")
            ser.write(f"{hat_print} {state_str}\n".encode())
        

        time.sleep(0.5)  # Delay to limit the message rate

except KeyboardInterrupt:
    print("Exiting...")
    pygame.joystick.quit()
    pygame.quit()