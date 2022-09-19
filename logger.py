import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

serialPort = serial.Serial(port = "COM9")
s = ""
current_temp = 0

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):
    global current_temp
    if(serialPort.in_waiting > 0):
        # Read data out of the buffer until a carraige return / new line is found
        serialString = serialPort.readline()
        
        s = serialString.decode('Ascii')
        if "TEMPERATURE" in s:
            s = s.split("TEMPERATURE :: ")[1]
            current_temp = int(s)

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%f'))
    ys.append(current_temp)

    # Limit x and y lists to 20 items
    xs = xs[-30:]
    ys = ys[-30:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('TMP102 Temperature over Time')
    plt.ylabel('Temperature (deg C)')

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
plt.show()