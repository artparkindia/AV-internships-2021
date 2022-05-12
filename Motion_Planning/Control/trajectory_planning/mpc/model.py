"""
Cost models for MPC Algorithm
"""
import numpy as np

class Model:
    """
    Base class for Models
    """
    def __init__(self, mpc):
        self.mpc = mpc

    def calculate_yaw(self, state):
        """ Function to calculate yaw """
        raise NotImplementedError

    def calculate_cost(self, x, xref):
        """ Cost Model """
        raise NotImplementedError

class BicycleModel(Model):
    """
    Bicycle Model
    """
    def __init__(self, mpc):
        self.mpc = mpc

    def calculate_yaw(self, state):
        """
        Function to calculate yaw
        """
        return state.yaw + state.v * np.tan(state.omega) * self.mpc.DT

    def calculate_cost(self, x, xref):
        """
        Bicycle cost model
        """
        t = self.mpc.T

        v_arr = x[ : t]
        omega_arr = x[t: 2*t] 

        cost=0
        x_arr = np.zeros(self.mpc.T)
        y_arr = np.zeros(self.mpc.T)
        yaw_arr = np.zeros(self.mpc.T)

        x,y,v,yaw = self.mpc.x,self.mpc.y,self.mpc.v,self.mpc.yaw
        for t in range(self.mpc.T):
            x += v*np.cos(yaw)*self.mpc.DT
            y += v*np.sin(yaw)*self.mpc.DT
            v = v_arr[t]
            yaw += v*np.tan(omega_arr[t])*self.mpc.DT
            x_arr[t] = x
            y_arr[t] = y
            yaw_arr[t] = yaw

        a_arr = v_arr[1:]-v_arr[:-1]

        cost += self.mpc.WEIGHTS[0] * np.sum((x_arr-xref[0][1:])**2)
        cost += self.mpc.WEIGHTS[1] * np.sum((y_arr-xref[1][1:])**2)
        cost += self.mpc.WEIGHTS[2] * np.sum((v_arr-xref[2][1:])**2)
        cost += self.mpc.WEIGHTS[3] * np.sum((yaw_arr-xref[3][1:])**2)

        cost += self.mpc.WEIGHTS[4] * np.sum(a_arr**2)
        cost += self.mpc.WEIGHTS[5] * np.sum((omega_arr[1:]-omega_arr[:-1])**2)

        cost += self.mpc.WEIGHTS[6] * np.sum((a_arr[1:]-a_arr[:-1])**2)

        return cost


class UnicycleModel:
    def calculate_yaw(self, state, mpc):
        """
        Function to calculate yaw
        """
        return state.yaw + state.omega

    def calculate_cost(self, x, xref):
        """
        Unicycle Cost Model
        """
        t = self.mpc.T

        v_arr = x[ : t]
        omega_arr = x[t: 2*t] 

        cost=0
        x_arr = np.zeros(self.mpc.T)
        y_arr = np.zeros(self.mpc.T)
        yaw_arr = np.zeros(self.mpc.T)
        

        x,y,v,yaw = self.mpc.x,self.mpc.y,self.mpc.v,self.mpc.yaw
        for t in range(self.mpc.T):
            x += v*np.cos(yaw)*self.mpc.DT
            y += v*np.sin(yaw)*self.mpc.DT
            v = v_arr[t]
            yaw += omega_arr[t]
            x_arr[t] = x
            y_arr[t] = y
            yaw_arr[t] = yaw

        a_arr = v_arr[1:]-v_arr[:-1]

        cost += self.mpc.WEIGHTS[0] * np.sum((x_arr-xref[0][1:])**2)
        cost += self.mpc.WEIGHTS[1] * np.sum((y_arr-xref[1][1:])**2)
        cost += self.mpc.WEIGHTS[2] * np.sum((v_arr-xref[2][1:])**2)
        cost += self.mpc.WEIGHTS[3] * np.sum((yaw_arr-xref[3][1:])**2)
        
        cost += self.mpc.WEIGHTS[4] * np.sum(a_arr**2)
        cost += self.mpc.WEIGHTS[5] * np.sum((omega_arr[1:]-omega_arr[:-1])**2)
        
        cost += self.mpc.WEIGHTS[6] * np.sum((a_arr[1:]-a_arr[:-1])**2)

        return cost