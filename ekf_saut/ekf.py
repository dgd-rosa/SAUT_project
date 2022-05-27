import numpy as np
from abc import ABC, abstractmethod
import random


class EKF(ABC):
    def __init__(self, state_dimension, process_mean, process_variance, measurement_mean, measurement_variance):
        self.state_dimension = state_dimension

        self.process_mean = process_mean
        self.process_variance = process_variance
        self.measurement_mean = measurement_mean
        self.measurement_variance = measurement_variance

    @abstractmethod
    def predict(self):
        pass

    @abstractmethod
    def update(self):
        pass

class EKF_simple(EKF):
    def __init__(self, state_dimension, process_mean, process_variance, measurement_mean, measurement_variance, altitude):
        self.state_dimension = state_dimension
        self.altitude = altitude # it is always constant
        self.current_apriori_state = [0] * state_dimension
        self.current_state = [0] * state_dimension

        self.process_mean = process_mean
        self.process_variance = process_variance
        self.measurement_mean = measurement_mean
        self.measurement_variance = measurement_variance

        self.P_priori = [0] * state_dimension

    def A_matrix(self):
        return np.identity(self.state_dimension)

    def Q_matrix(self):
        tmp_vect = np.full(shape=self.state_dimension, fill_value = (random.gauss(self.process_mean, self.process_variance))**2, dtype=np.float)
        return np.diag(tmp_vect)
    
    def R_matrix(self):
        tmp_vect = np.full(shape=self.state_dimension, fill_value = (random.gauss(self.measurement_mean, self.process_variance))**2, dtype=np.float)
        return np.diag(tmp_vect)
    
    def H_matrix(self, x_beacon, y_beacon, psi, b, e):
        x_k = self.current_apriori_state[0]
        y_k = self.current_apriori_state[1]
        aux_x = (x_k - x_beacon)/np.sqrt((x_k-x_beacon)**2 + (y_k-y_beacon)**2 + self.altitude**2) # we consider z_beacon=0
        aux_y = (y_k - y_beacon)/np.sqrt((x_k-x_beacon)**2 + (y_k-y_beacon)**2 + self.altitude**2) # we consider z_beacon=0
        return np.array([np.cos(-e)*np.cos(-b-psi)*aux_x , np.cos(-e)*np.cos(-b-psi)*aux_y], [np.cos(-e)*np.sin(-b-psi)*aux_x , np.cos(-e)*np.sin(-b-psi)*aux_y])

    #TODO ver ruído pois ruido = v R v^T
    def kalman_gain(self, H, R):
        return self.P_priori @ np.transpose(H) @ np.inverse(H @ self.P_priori @ np.transpose(H) + R)

    def predict(self, u, v, psi, t_step):
        # Compute the predicted state
        self.current_apriori_state[0] = self.current_state[0] + u*t_step*np.cos(psi) - v*t_step*np.sin(psi)
        self.current_apriori_state[1] = self.current_state[1] + u*t_step*np.sin(psi) + v*t_step*np.cos(psi)

        # Compute the a priori error covariance estimate
        self.P_priori = self.A_matrix() @ self.P_priori @ np.transpose(self.A_matrix()) + self.Q_matrix()

    #TODO: Ended my last work here in update function
    def update(self, x_beacon, y_beacon, psi, b, e):
        x_k = self.current_apriori_state[0]
        y_k = self.current_apriori_state[1]
        
        h_x = x_beacon + np.cos(-e)*np.cos(-b-psi)*np.sqrt((x_k-x_beacon)**2 + (y_k-y_beacon)**2 + self.altitude**2) # we consider z_beacon=0
        h_y = y_beacon + np.cos(-e)*np.sin(-b-psi)*np.sqrt((x_k-x_beacon)**2 + (y_k-y_beacon)**2 + self.altitude**2) # we consider z_beacon=0
        h = np.array([z_x], [z_y])
        
        K = self.kalman_gain(self.H_matrix(x_beacon, y_beacon, psi, b, e), self.R_matrix())
        #x_k+1 = x_apriori + K(z - h(x_k))
        self.current_state = self.current_apriori_state + K*(z - )

    # Beacon X and Y as input args, as these could be later dynamic... for now we consider the beacon as always in the same position
    def compute_iteration(self, x_beacon, y_beacon, u, v, r, b, e, psi, t_step):
        self.predict(u, v, r, b, e, psi, t_step)
        self.update(x_beacon, y_beacon)
        print(self.current_state)


# The main here is just for testing... later on the EKF will be called from a ROS Node
if __name__ == '__main__':
    beacon_x = 2
    beacon_y = 3

    test_sensor_readings = np.matrix([[0, 1, 2, 3], [4, 5, 6, 7], [2, 2, 2, 2]]) #[r, b, e, psi]
    test_inputs = np.matrix([[0, 0], [1, 1], [2, 2]]) #[u, v]
    ekf = EKF_simple(2, 0, 0.01, 0, 0.01)

    #I wanna test for 10 EKF iterations
    for i in range(3):
        ekf.compute_iteration(beacon_x, beacon_y, test_inputs[i][0], test_inputs[i, 1], test_sensor_readings[i, 0], test_sensor_readings[i, 1], test_sensor_readings[i, 2], test_sensor_readings[i, 3])
