import numpy as np
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

class SillyController:
    def get_control(self,T,dt):
        return 0
silly_controller = SillyController()
simulate_temp(silly_controller, num_steps=30)
