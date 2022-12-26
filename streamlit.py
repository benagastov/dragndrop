import numpy as np
import streamlit as st
import matplotlib.pyplot as plt
from IPython import display

# Create tabs
tab0, tab1, tab2, tab3 = st.tabs(["Dummy","Proportional", "Integral", "PID"])

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
with tab0:
    st.header("Dummy Controller")
    st.markdown("Imagine that we have prepared a solar cell to improve its performance, but the temperature of the solar cell is more than 29°C or less than 20°C and should be 25°C. The ambient temperature in our room is 19°C and this will reduce the performance or even damage the solar cell. Luckily, we have a device that can heat up the solar cell. The device uses electricity to produce heat and we can manipulate the power $u$. We are also able to monitor the temperature of the solar cell $T$. What we are facing is a performance optimization problem.")
    st.markdown("Before we explore a solution, let us write some code to simulate the scenario. We will model the temperature dynamics through Newton's law of cooling.")
    st.latex(r'''
    \frac{dT}{dt}=\alpha (T_a - T(t)) + \beta u(t)
    ''')
    st.markdown("Here, $d/dt$ is the time derivative, $T_a$ is the ambient temperature and $u$ the power of the heating device. \alpha and \beta are constants. We are not going to try to estimate realistic values for \alpha and \beta for our problem. This is still ideal environment. Using the equation above, let us write some simulation code. ")
    st.markdown("The `simulate_temp` function needs a `controller` object as an input. This `controller` has a function `get_control()`, which looks at the current temperature `T` and the time `dt` that has elapsed since the last control command was issued. It tells us to which power $u$ we should set our cooling or heating device.")
    st.markdown("First let us create a very silly controller: It will always set $u$ to zero. Therefore, we are not activating the cooling or heating device and it will cool down or heat up to the ambient temperature:")
    silly_controller = SillyController()
    simulate_temp(silly_controller, num_steps=30)
    st.markdown(":Indeed, if we do nothing, the solar panel cools down. But now let us try to create a proper controller. ")

class PController:
    def __init__(self, Kp, set_point):
        self.Kp = Kp
        self.set_point = set_point
    
    def get_control(self, measurement, dt):
        error = self.set_point - measurement
        return self.Kp * error

with tab1:
    st.header("Proportional Controller")
    st.markdown("The idea of the P controller or proportional controller is simple: Imagine that we are turning a knob on the heating device that sets the value of $u$. We look at the difference between the desired temperature and the current temperature, the so-called error. If the error is large and positive (desired temperature > current temperature) we choose $u$ to be large and positive. This will heat up the solar panel and the error will go down. The more the error goes down, the more we would turn the $u$-knob towards zero. If the error is negative, i.e., the current solar panel temperature is too high (dangerous for the photovoltaic cell!), we would like to cool the panel down by setting $u$ to a negative value. Unfortunately, this is something we cannot do with our electrical heater. ")
    st.markdown("We can imagine other control problems, where the control input $u$ does not need to be positive. For example we might want to control the torque on the wheels of a car in order to park at a specific position. If we have driven too far, we can set a negative torque and drive backwards. ")
    st.markdown("The mathematical formula for the P controller is: ")
    st.latex(r'''
    $$u(t) = K_p e(t)$$
    ''')
    st.markdown("Here, $e$ denotes the error. Now, let us apply the `PController` and see what happens to the solar:")
    pi_controller = PIController(Kp=0.2, Ki = 0.15, set_point=T_desired)
    simulate_temp(pi_controller)
    st.markdown("As you can see, the solar panel's temperature goes up, but not quite to the value we wanted. There is still a gap between the actual temperature and the desired temperature. This is known as steady state error and is a typical problem with a `PController`.  ")
    st.markdown("In a real system it can be hard to understand why there is a *steady state error*, because we might not even have a model of the system. For our simulated solar panel system however, we can understand it: Assume that the actual temperature is equal to the desired temperature. Then the error `T_desired-T` is zero and hence `u=K_d * (T_desired-T)` is zero. This means no heat is added and the solar panel cools down below `T==T_desired`. A steady state is reached when the heat generated by the error is equal to the heat that is lost to the room. ")
    st.markdown("Now we could increase the `set_point` to desired temperature plus a bit more. This would increase the steady state temperature and make it come closer to our desired temperature. But there exists a much better solution! The proportional integral or PI controller!  ")

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
    st.markdown(":Indeed, if we do nothing, the solar panel cools down. But now let us try to create a proper controller. ")
    pi_controller = PIController(Kp=0.2, Ki = 0.15, set_point=T_desired)
    simulate_temp(pi_controller)
    st.markdown(":Indeed, if we do nothing, the solar panel cools down. But now let us try to create a proper controller. ")



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


