#print("\n" * 100)
import numpy as np
#import matplotlib.pyplot as plt
import plotly.graph_objects as go

class PID:
    """ PID Controller """
    def __init__(self, P, I, D):
        self.Kkp = P
        self.Kki = I
        self.Kkd = D
        self.integral = 0
        self.prev_error = 0

    def update(self, setpoint, pv, dt):
        error = setpoint - pv
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kkp * error + self.Kki * self.integral + self.Kkd * derivative
        self.prev_error = error
        return output

# Constants
max_voltage = 12.0  # Volts
max_speed = 120.0   # rpm

def voltage_to_speed(voltage):
    """ Convert voltage to motor speed """
    return (voltage / max_voltage) * max_speed

def scale_output_to_voltage(pid_output, min_pid_output, max_pid_output, min_voltage, max_voltage):
    # Normalize the PID output to a 0-1 range
    normalized_output = (pid_output - min_pid_output) / (max_pid_output - min_pid_output)
    # Scale to the voltage range
    voltage = normalized_output * (max_voltage - min_voltage) + min_voltage
    # Ensure the voltage does not exceed limits (optional, as the formula should handle it)
    voltage = max(min_voltage, min(voltage, max_voltage))
    return voltage



# Simulation parameters
dt = 0.1  # Time step
total_time = 20  # Total simulation time in seconds
times = np.arange(0, total_time, dt)

##Initial parameter calculations:
Kc = 0.1  #Critical gain
Pc = 0.2 #oscillation period in s
Kp = 0.6 * Kc #
Ti = 0.5 * Pc #=0.5*0.2 = 0.1
Td = 0.125 * Pc
Ki = Kp/Ti #= 0.6*0.1/0.1 
Kd = Kp * Td
print("Initial parameters")
print("Kp= ", Kp)
print("Ki= ", Ki)
print("Kd= ", Kd)
####################################

# Initialize PID controller
# pid = PID(P=Kp, I=Ki, D=Kd)
P = Kp/24
I=Ki
D=Kd
pid = PID(P=P, I=I, D=D)
#PID parameters
print("PID parameters")
print("P= ", P)
print("I= ", I)
print("D= ", D)


# Target and chaser initial conditions
current_target_speed = 0.1 #rpm
current_chaser_speed = 0
chaser_mass = 1 #kg
chaser_radius = 0.1 #m
chaser_inertia = 0.5 * chaser_mass * (chaser_radius**2) #kgm^2


# For plotting
target_speed = []
chaser_speed = []
target_speed.append(current_target_speed)
chaser_speed.append(current_chaser_speed)

max_voltage = 12
min_voltage = 0

for t in times:    
    # PID control
    pid_output = pid.update(current_target_speed, current_chaser_speed, dt)
#     print("pid_output= ",pid_output)
    #voltage = scale_output_to_voltage(pid_output, Min_output, Max_output, min_voltage, max_voltage)
    voltage = pid_output
    voltage = max(0, min(max_voltage, voltage))  # Clamp voltage to 0-12V
    current_chaser_speed = voltage_to_speed(voltage)  # Convert voltage to speed
    # Store data for plotting
    target_speed.append(current_target_speed)
    chaser_speed.append(current_chaser_speed)


# # Plotting the results
# import matplotlib.pyplot as plt
# 
# plt.figure(figsize=(10, 5))
# plt.plot(times, target_positions, label='Target Position')
# plt.plot(times, chaser_positions, label='Chaser Position')
# plt.xlabel('Time (s)')
# plt.ylabel('Position (degrees)')
# plt.title('PID Controller Tracking Simulation')
# plt.legend()
# 
# # Adding gridlines
# plt.grid(True, which='both', linestyle='--', linewidth=0.5)
# 
# plt.show()


import plotly.graph_objects as go


# Assuming you have your times, target_positions, and chaser_positions arrays ready
fig = go.Figure()

# Add traces for target and chaser positions
fig.add_trace(go.Scatter(x=times, y=target_speed, mode='lines', name='Target Speed'))
fig.add_trace(go.Scatter(x=times, y=chaser_speed, mode='lines', name='Chaser Speed'))

P_value = pid.Kkp
I_value = pid.Kki
D_value = pid.Kkd

fig.add_annotation(x=max(times)*0.8, y=max(target_speed), text=f"P: {P_value}, I: {I_value}, D: {D_value}",
                   showarrow=False, font=dict(family="Courier New, monospace", size=16, color="#ffffff"),
                   align="left", bgcolor="#000000", bordercolor="#ffffff", borderwidth=2, borderpad=4)



# Add titles and labels
fig.update_layout(
    title='PID Controller Tracking Simulation',
    xaxis_title='Time (s)',
    yaxis_title='Speed (rpm)',
    template='plotly_dark',  # Optional: for a dark theme
    xaxis=dict(
        showline=True,
        showgrid=True,
        showticklabels=True,
        linewidth=2,
        ticks='outside',
        tickfont=dict(
            family='Arial',
            size=12,
            color='rgb(82, 82, 82)',
        ),
    ),
    yaxis=dict(
        showgrid=True,  # This automatically adds horizontal gridlines
        zeroline=False,
        showline=False,
        showticklabels=True,
    )
)

# Show the figure
fig.show()