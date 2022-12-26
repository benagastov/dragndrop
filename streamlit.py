import numpy as np
import streamlit as st
import matplotlib.pyplot as plt
from IPython import display

# Create tabs
tab1, tab2, tab3 = st.tabs(["Proportional", "Integral", "PID"])

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


class SillyController:
    def get_control(self,T,dt):
        return 0
with tab1:
    st.header("Proportional Controller")
    st.markdown("Imagine that we have prepared a solar cell to improve its performance, but the temperature of the solar cell is more than 29°C or less than 20°C and should be 25°C. The ambient temperature in our room is 19°C and this will reduce the performance or even damage the solar cell. Luckily, we have a device that can heat up the solar cell. The device uses electricity to produce heat and we can manipulate the power $u$. We are also able to monitor the temperature of the solar cell $T$. What we are facing is a performance optimization problem.")
    st.markdown("Before we explore a solution, let us write some code to simulate the scenario. We will model the temperature dynamics through Newton's law of cooling.")
    st.markdown("\frac{dT}{dt}=\alpha (T_a - T(t)) + \beta u(t)")
    st.markdown("Here, $d/dt$ is the time derivative, $T_a$ is the ambient temperature and $u$ the power of the heating device. $\alpha$ and $\beta$ are constants. We are not going to try to estimate realistic values for $\alpha$ and $\beta$ for our problem. This is still ideal environment. Using the equation above, let us write some simulation code. ")
    silly_controller = SillyController()
    simulate_temp(silly_controller, num_steps=30)


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

with tab2:
    st.header("PI Controller")
    pi_controller = PIController(Kp=0.2, Ki = 0.15, set_point=T_desired)
    simulate_temp(pi_controller)



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



with tab3:
    col1, col2 = st.columns([1, 3])
    with col1:
        st.markdown("### Constants Controls")
        st.markdown("You can **change** the values to change the *graph*.")
        Kp =  st.slider("Kp", min_value=0.0, max_value=0.4, value=0.1, step=0.05)
        Ki =  st.slider("Ki", min_value=0.0, max_value=0.5, value=0.1, step=0.01)
        Kd =  st.slider("Kd", min_value=-3e-3, max_value=3e-3, value=-3e-3, step=2e-4)

    def run_pid_temp(Kp,Ki,Kd):
        pid_controller = PIDController(Kp, Ki, Kd, set_point=T_desired)
        simulate_temp(pid_controller)
    with col2:
        st.header("PID Controller")
        run_pid_temp(Kp,Ki,Kd)


