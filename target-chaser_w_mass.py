import numpy as np
import plotly.graph_objects as go
import plotly.offline as pyo

class PID: 
    """ PID Controller """
    def __init__(self, P, I, D):
        self.Kkp = P #Proportional term
        self.Kki = I #Integration term
        self.Kkd = D #Derivative term
        self.integral = 0 #initialising integral term to 0
        self.prev_error = 0 #initialising error term to 0

    def update(self, setpoint, pv, dt):
        error = setpoint - pv #error term, pv= process variable
        self.integral += error * dt #integral term, dt = timestep
        derivative = (error - self.prev_error) / dt #derivative term
        output = self.Kkp * error + self.Kki * self.integral + self.Kkd * derivative #PID output calculation
        self.prev_error = error #updating the previous error term (for next calculation) to calculated current error
        return output

# Constants
max_voltage = 12.0  # Motor max volts
max_speed = 120.0   # motor max rpm

def voltage_to_speed(voltage):
    """ Convert voltage to motor speed """
    return (voltage / max_voltage) * max_speed # Motor speed is controlled by a 0-12 volt signal. The motor does not have a reverse direction rotation in this simulation.

# Simulation parameters
dt = 0.001  # Time step
total_time = 4  # Total simulation time in seconds
times = np.arange(0, total_time, dt)

#PID parameters for calculations:
Kc = 12.9999 #Critical gain
Pc = 0.0012 #oscillation period in s
Kp = 0.6 * Kc #proportional parameter relationship with proprtionality parameter
Ti = 0.5 * Pc 
Td = 0.125 * Pc
Ki = Kp/Ti #integral parameter relationship with proprtionality parameter
Kd = Kp * Td #derivative parameter relationship with proprtionality parameter
# print("Initial parameters")
# print("Kp= ", Kp)
# print("Ki= ", Ki)
# print("Kd= ", Kd)
####################################

#parameters to be set
P = Kp*20
I= Ki/10
D= Kd
# P = Kp*5
# I= Ki/10
# D= Kd
pid = PID(P=P, I=I, D=D)
#PID parameters
# print("PID parameters")
print("P= ", P)
print("I= ", I)
print("D= ", D)


# Target and chaser initial conditions
current_target_speed = 40 #rpm #target motor speed (0 to 120 rpm range)
initial_voltage = 0 #V #initial control signal voltage (0 to 12 V range)
current_chaser_speed = 0 #rpm # initial chaser motor speed, can be set to any value.
chaser_mass = 2 #kg #chaser motor rotational mass
chaser_radius = 0.1 #m #for radius of gyraion for the chaser motor rotational mass
chaser_inertia = chaser_mass * (chaser_radius**2) #kgm^2 #chaser motor rotational inertia
chaser_torque_max = 0.5 #Nm #max possible torque at max chaser motor speed
chaser_load = 0.5 #Nm #chaser motor resistance torque, combination of friction, load torque
chaser_power = (2*np.pi*max_speed*chaser_torque_max)/60 #Watts #chaser motor power calculated from max speed and max torque at max speed
# print(chaser_power)

chaser_load_speed = ((60*chaser_load*dt)/(chaser_inertia*2*np.pi)) #rpm #speed reduction due to load

def torque_calc(speed,power): #torque calculator at any speed for max power. Motor brake power is assumed to be constant.
    speed = max(speed, 1) #min speed requirement, any speed values below 1 rpm are ceiled to 1 rpm to avoid extremely high torque numerical anamoly.
    torque = (60*power)/(2*speed*np.pi) 
    return torque

# torq = torque_calc(0,chaser_power)
# print (torq)
# print(((60*torq*dt)/(chaser_inertia*2*np.pi))-((60*chaser_load*dt)/(chaser_inertia*2*np.pi)))

target_speed = [] #List to store target motor speed values, used for plotting data
chaser_speed = [] #List to store chaser motor speed values, used for plotting data
voltages = [] #List to store control signal voltage values, used for plotting data
integrals =[] #list to store integral terms values, used for plotting data
target_speed.append(current_target_speed) #appending initial target motor speed
chaser_speed.append(current_chaser_speed) #appending initial chaser motor speed
voltages.append(initial_voltage) #appending initial voltage signal
integrals.append(0) #appending integral to 0

max_voltage = 12 #max control signal voltage
min_voltage = 0 #min control signal voltage

#Control Loop
for t in times:           
    # PID control
    voltage = pid.update(current_target_speed, current_chaser_speed, dt) #PID function for voltage estimation
    integrals.append(pid.integral) #storing integral terms for plotting
    voltage = max(0, min(max_voltage, voltage))  # Clamp voltage to 0-12V
    voltages.append(voltage)  # Store voltage for plotting
    voltage_chaser_speed = voltage_to_speed(voltage)  # speed set by voltage
    #Speed reduction due to resisting load:
    
    #correcting/ boosting chaser speed:
    if current_chaser_speed <= voltage_chaser_speed: #condition for control signal input to correct speed in case it is lower than expected
        chaser_torque = torque_calc(current_chaser_speed, chaser_power) #available torque to chaser motor based on max power and current speed    
        current_chaser_speed = current_chaser_speed + ((60*(chaser_torque*(voltage/max_voltage))*dt)/(chaser_inertia*2*np.pi)) - chaser_load_speed #speed correction with drive torque and load torque with consideration of inertia
        current_chaser_speed = max(current_chaser_speed, 0) #clamping chaser speed to 0, and avoiding negative speeds (extra resistance causes the motor to stall)
    else:
       current_chaser_speed -= ((60*(chaser_load*dt/chaser_inertia))/(2*(np.pi))) #if no correction is required (motor speed more than required speed), resistance force slows down the motor
       current_chaser_speed = max(current_chaser_speed, 0) #clamping chaser speed to 0, and avoiding negative speeds (extra resistance causes the motor to stall)
    # Store data for plotting
    target_speed.append(current_target_speed) #storing target speed for plotting
    chaser_speed.append(current_chaser_speed) #storing chaser speed for plotting

import plotly.graph_objects as go

fig = go.Figure()

fig.add_trace(go.Scatter(x=times, y=target_speed, mode='lines', name='Target Speed')) #trace for target speed
fig.add_trace(go.Scatter(x=times, y=chaser_speed, mode='lines', name='Chaser Speed')) #trace for chaser speed
#fig.add_trace(go.Scatter(x=times, y=voltages, mode='lines', name='Voltage', line=dict(color='yellow'), yaxis='y2')) #trace for voltage
fig.add_trace(go.Scatter(x=times, y=voltages, mode='lines', name='Voltage')) #trace for voltage
fig.add_trace(go.Scatter(x=times, y=integrals, mode='lines', name='integrals')) #trace for chaser speed

P_value = pid.Kkp #retrieving PID values to print on the chart
I_value = pid.Kki #retrieving PID values to print on the chart
D_value = pid.Kkd #retrieving PID values to print on the chart

fig.add_annotation(x=max(times)*0.8, y=max(target_speed), text=f"P: {P_value}, I: {I_value}, D: {D_value}",
                   showarrow=True, font=dict(family="Courier New, monospace", size=16, color="#ffffff"),
                   align="left", bgcolor="#000000", bordercolor="#ffffff", borderwidth=2, borderpad=4)  #annotating PID values on the chart



# Add titles and labels
fig.update_layout(
    title='PID Controller Tracking Simulation',
    xaxis_title='Time (s)',
    yaxis_title='Speed (rpm)',
    yaxis=dict(
        title='Speed (rpm)',
        titlefont=dict(color="#1f77b4"),
        tickfont=dict(color="#1f77b4"),
        showgrid=True,  # horizontal gridlines
        zeroline=False,
        showline=False,
        showticklabels=True,
    ),
#     yaxis2=dict(
#         title='Voltage (V)',
#         titlefont=dict(color="#ff7f0e"),
#         tickfont=dict(color="#ff7f0e"),
#         anchor='x',
#         overlaying='y',
#         side='right',
#         range=[0, 120]  # Setting the range for voltage
#     ),
    template='plotly_dark',  # Optional: for a dark theme
)

# Show the figure
pyo.plot(fig, filename='pid_controller_tracking_simulation.html')