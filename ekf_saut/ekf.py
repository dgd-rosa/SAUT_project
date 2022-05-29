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
    def __init__(self, state_dimension, measurement_dimension, process_mean, process_variance, measurement_mean, measurement_variance, altitude):
        self.state_dimension = state_dimension
        self.measurement_dimension = measurement_dimension
        self.altitude = altitude # it is always constant
        self.current_apriori_state = np.zeros((state_dimension, 1))
        self.current_state = np.zeros((state_dimension, 1))

        self.process_mean = process_mean
        self.process_variance = process_variance
        self.measurement_mean = measurement_mean
        self.measurement_variance = measurement_variance

        self.P_apriori = np.zeros((state_dimension, 1))
        self.P_aposteriori = np.zeros((state_dimension, 1))

    def A_matrix(self):
        return np.identity(self.state_dimension)

    def Q_matrix(self):
        tmp_vect = np.ones((self.state_dimension, 1)) * ((random.gauss(self.process_mean, self.process_variance))**2)
        return tmp_vect
    
    def R_matrix(self):
        tmp_vect = np.ones((self.measurement_dimension, 1)) * ((random.gauss(self.process_mean, self.process_variance))**2)
        return tmp_vect
    
    def H_matrix(self, x_beacon, y_beacon, psi, b, e):
        x = self.current_apriori_state[0] - x_beacon
        y = self.current_apriori_state[1] - y_beacon

        aux_sqrt = np.sqrt((x)**2 + (y)**2 + self.altitude**2) # we consider z_beacon=0

        return np.array([[ (x / aux_sqrt), (y / aux_sqrt) ], [ (y / (x**2 + y**2)), (-x / (x**2 + y**2)) ], [ ((-self.altitude*(2*x**2 + y**2)) / (np.sqrt(x**2 + y**2) * (x**4 + x**2 * y**2 + self.altitude**2)) ), ((-x*y*self.altitude) / (np.sqrt(x**2 + y**2) * (x**2 * y**2 + y**4 + self.altitude**2)))]])

    #TODO ver ruído pois ruido = v R v^T
    def kalman_gain(self, H, R):
        return self.P_apriori @ np.transpose(H) @ np.linalg.inv(H @ self.P_apriori @ np.transpose(H) + R)

    def predict(self, u, v, psi, t_step):
        # Compute the predicted state
        #print(u)
        #print(u*t_step*np.cos(psi))
        #print(v*t_step*np.sin(psi))

        
        self.current_apriori_state[0] = self.current_state[0] + u*t_step*np.cos(psi) - v*t_step*np.sin(psi)
        self.current_apriori_state[1] = self.current_state[1] + u*t_step*np.sin(psi) + v*t_step*np.cos(psi)

        # Compute the a priori error covariance estimate
        print(self.A_matrix())
        print("\n")
        print(self.P_aposteriori)
        print("\n")
        print(np.transpose(self.A_matrix()))
        print("\n")
        print(self.Q_matrix())
        print("\n")
        print(self.P_apriori)
        print("\n")
        print(np.transpose(self.A_matrix()))
        print("Cavani")
        print(self.A_matrix() @ self.P_aposteriori)
        print("Ola")
        #print(self.A_matrix() @ self.P_aposteriori @ np.transpose(self.A_matrix()))
        self.P_apriori = (self.A_matrix() @ self.P_aposteriori @ np.transpose(self.A_matrix())) + self.Q_matrix()

    #TODO: Ended my last work here in update function
    def update(self, x_beacon, y_beacon, psi, b, e, measures):
        x_k = self.current_apriori_state[0]
        y_k = self.current_apriori_state[1]
        
        h_r = np.sqrt((x_k - x_beacon)**2 + (y_k - y_beacon)**2 + self.altitude**2) # we consider z_beacon=0
        h_b = -np.arctan((y_k - y_beacon) / (x_k - x_beacon)) - psi
        h_e = np.arctan(-self.altitude / (np.sqrt((x_k - x_beacon)**2 + (y_k - y_beacon)**2)))
        h = np.array([[h_r], [h_b], [h_e]])
        
        K = self.kalman_gain(self.H_matrix(x_beacon, y_beacon, psi, b, e), self.R_matrix())
        #x_k+1 = x_apriori + K(z - h(x_k))
        self.current_state = self.current_apriori_state + K*(measures.T - h)

        self.P_aposteriori = (np.identity(self.state_dimension) - K@h)@self.P_apriori

    # Beacon X and Y as input args, as these could be later dynamic... for now we consider the beacon as always in the same position
    def compute_iteration(self, x_beacon, y_beacon, u, v, psi, measures, t_step):
        self.predict(u, v, psi, t_step)
        self.update(x_beacon, y_beacon, psi, measures[0, 1], measures[0, 2], measures)
        #print(self.current_state)


# The main here is just for testing... later on the EKF will be called from a ROS Node
if __name__ == '__main__':
    beacon_x = 2
    beacon_y = 3

    t_step = 1
    altitude = 5
    measures = np.matrix([[0, 1, 2], [4, 5, 6], [2, 2, 2]]) #[r, b, e]
    test_inputs = np.matrix([[0, 0], [1, 1], [2, 2]]) #[u, v]
    ekf = EKF_simple(2, 3, 0, 0.01, 0, 0.01, altitude)

    #I wanna test for 10 EKF iterations
    for i in range(3):
        ekf.compute_iteration(beacon_x, beacon_y, test_inputs[i, 0], test_inputs[i, 1], 0.2, measures[i], t_step)
