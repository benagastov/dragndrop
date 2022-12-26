import numpy as np
import streamlit as st
import matplotlib.pyplot as plt
from IPython import display

#define constants
alpha, beta = 1, 35
T_ambient, T_desired, T_start = 18, 25, 21

# system update by discretization of the differential eq.
def next_temp(u, T, dt):
    return T+alpha*(T_ambient-T)*dt + beta * u *dt

def simulate_temp(controller, num_steps=20):
    fig = plt.figure() 
    dt = 0.1 # Every time interval dt we set a new control value
    T = T_start
    T_list = [T]
    for k in range(num_steps):
        # ask controller for u value
        u = controller.get_control(T,dt)
        # device only allows to set u between 0 and 1:
        u = np.clip(u, 0, 1)
        # simulate what the temperature will be after time interval dt
        T = next_temp(u, T, dt)
        T_list.append(T)
    time = dt*np.arange(num_steps+1)
    plt.plot(time, np.repeat(T_desired, num_steps+1), ls="--")
    plt.plot(time, T_list)
    plt.xlabel("time"); plt.ylabel("Temperature");
    st.pyplot(fig)

# Create tabs
tab1, tab2, tab3 = st.tabs(["Proportional", "Integral", "Derivative"])

with tab1:
    st.header("Proportional Controller")
    class SillyController:
        def get_control(self,T,dt):
            return 0
    silly_controller = SillyController()
    simulate_temp(silly_controller, num_steps=30)

with tab2:
    st.header("Integral Controller")
    class PIController:
        def __init__(self, Kp, Ki, set_point):
            self.Kp = Kp
            self.Ki = Ki
            self.set_point = set_point
            self.int_term = 0
    
    def get_control(self, measurement, dt):
        error = self.set_point - measurement
        self.int_term += error*self.Ki*dt
        return self.Kp * error + self.int_term
    pi_controller = PIController(Kp=0.2, Ki = 0.15, set_point=T_desired)
    simulate_temp(pi_controller)

with tab3:
    st.header("PID Controller")
    class PIDController:
        def __init__(self, Kp, Ki, Kd, set_point):
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.set_point = set_point
            self.int_term = 0
            self.derivative_term = 0
            self.last_error = None
    
    def get_control(self, measurement, dt):
        error = self.set_point - measurement
        self.int_term += error*self.Ki*dt
        if self.last_error is not None:
            self.derivative_term = (error-self.last_error)/dt*self.Kd
        self.last_error = error
        return self.Kp * error + self.int_term + self.derivative_term

# create sliders for interactive simulation via python
# Does not work inside Jupyter Book; only in notebook environment
from ipywidgets import interact, FloatSlider
import ipywidgets as widgets
def run_pid_temp(Kp,Ki,Kd):
    pid_controller = PIDController(Kp, Ki, Kd, set_point=T_desired)
    simulate_temp(pid_controller)

Kp_slider = FloatSlider(min=0.0, max=0.8, step=0.05, readout_format='.4f')
Ki_slider = FloatSlider(min=0.0, max=0.5, step=0.01, readout_format='.4f')
Kd_slider = FloatSlider(min=-3e-3, max=3e-3, step=2e-4, readout_format='.4f')
interact(run_pid_temp, Kp=Kp_slider, Ki=Ki_slider, Kd=Kd_slider);
