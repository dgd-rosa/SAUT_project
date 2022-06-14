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
        self.current_state = np.zeros((state_dimension, 1))

        self.process_mean = process_mean
        self.process_variance = process_variance
        self.measurement_mean = measurement_mean
        self.measurement_variance = measurement_variance

        tmp_vector = np.ones(state_dimension) * 0.01
        self.P_apriori = np.identity(self.state_dimension) * ((random.gauss(self.process_mean, self.process_variance))**2)
        self.P_aposteriori = (np.identity(self.state_dimension) * ((random.gauss(self.process_mean, self.process_variance))**2))
        #self.P_aposteriori = np.diag(tmp_vector)

    def A_matrix(self):
        return np.identity(self.state_dimension)

    def Q_matrix(self):
        #return (np.identity(self.state_dimension) * (self.process_variance))
        return (np.identity(self.state_dimension) * ((random.gauss(self.process_mean, self.process_variance))**2))
    
    def R_matrix(self):
        #return (np.identity(self.measurement_dimension) * (self.measurement_variance))
        return (np.identity(self.measurement_dimension) * ((random.gauss(self.process_mean, self.process_variance))**2))
    
    def H_matrix(self, x_beacon, y_beacon):
        #x = x_beacon - self.current_state[0,0]
        #y = y_beacon - self.current_state[1,0]

        x = self.current_state[0,0]
        y = self.current_state[1,0]

        aux_sqrt = np.sqrt((x - x_beacon)**2 + (y - y_beacon)**2 + self.altitude**2) # we consider z_beacon=0
        #Range Derivatives
        a = ( (x - x_beacon) / aux_sqrt)
        b = ( (y - y_beacon) / aux_sqrt)
        #Elevation derivatives
        c = -((self.altitude * (x - x_beacon)) / ( np.sqrt((x - x_beacon)**2 + (y_beacon - y)**2) * ( (x - x_beacon)**2 + (y_beacon - y)**2 + self.altitude**2 ) )) # Actually it is - (-self.altitude * x_inv) / ( np.sqrt(x_inv**2 + y_inv**2) * ( x_inv**2 + y_inv**2 + c**2 - 2*z_beaco*z + z**2 ) ) But the ship is always at z=0
        d = -((self.altitude * (x - x_beacon)) / ( np.sqrt((x - x_beacon)**2 + (y_beacon - y)**2) * ( (x - x_beacon)**2 + (y_beacon - y)**2 + self.altitude**2 ) )) # Same here
        #Bearing derivatives
        e = ( (y_beacon - y) / ((x - x_beacon)**2 + (y_beacon - y)**2))
        f = ( (x - x_beacon) / ((x - x_beacon)**2 + (y_beacon - y)**2))

        return np.array([[ a,  b], [c, d], [e, f]]).reshape(3, 2)

    def kalman_gain(self, H, R):
        
        return self.P_apriori @ np.transpose(H) @ np.linalg.inv(H @ self.P_apriori @ np.transpose(H) + R)
        

    def predict(self, u, v, psi, t_step):
        # Compute the predicted stateelf.current_state[0, 0]
        self.current_state[0, 0] = self.current_state[0, 0] + u*t_step*np.cos(psi) - v*t_step*np.sin(psi)
        self.current_state[1, 0] = self.current_state[1, 0] + u*t_step*np.sin(psi) + v*t_step*np.cos(psi)

        # Compute the a priori error covariance estimate
        self.P_apriori = (self.A_matrix() @ self.P_aposteriori @ np.transpose(self.A_matrix())) + self.Q_matrix()
        

    # measures = [RANGE, ELEVATION, BEARING]
    def update(self, x_beacon, y_beacon, psi, measures):
        x_k = self.current_state[0, 0]
        y_k = self.current_state[1, 0]
        print("Antes do update:")
        print("Current State: " + str(self.getCurrent_State()))
        print("A Posteriori: " + str(self.P_aposteriori))
        print("*********")

        h_r = np.sqrt((x_k - x_beacon)**2 + (y_beacon - y_k)**2 + self.altitude**2) # we consider z_beacon=0
        h_b = np.arctan2((y_beacon - y_k), (x_beacon - x_k))
        h_e = np.arctan2(-self.altitude, (np.sqrt((x_beacon - x_k)**2 + (y_beacon - y_k)**2)))
        
        
        h = np.array([h_r, h_e, h_b])
        h = np.transpose(h)
        print("h: " + str(h) + "\n")
        print("measures: " + str(measures) + "\n")
        
        H = self.H_matrix(x_beacon, y_beacon)
        K = self.kalman_gain(H, self.R_matrix())
        print("K: " + str(K) + "\n")
        print("K(y-h): " + str(K@(measures - h)) + "\n" )

        self.current_state[:,0] = self.current_state[:,0] + K@(measures - h)
        self.P_aposteriori = (np.identity(self.state_dimension) - K@H) @ self.P_apriori


    # Beacon X and Y as input args, as these could be later dynamic... for now we consider the beacon as always in the same position
    def compute_iteration(self, x_beacon, y_beacon, u, v, psi, measures, t_step):
        self.predict(u, v, psi, t_step)
        self.update(x_beacon, y_beacon, psi, measures)
    
    def getCurrent_State(self):
        return self.current_state
    def getCurrentAprioriState(self):
        return self.current_state
    def getCovariance(self):
        return self.P_aposteriori
    def getAprioriCovariance(self):
        return self.P_apriori


# The main here is just for testing... later on the EKF will be called from a ROS Node
if __name__ == '__main__':
    beacon_x = 2
    beacon_y = 3

    t_step = 1
    altitude = 5
    measures = np.array([[0, 1, 2], [4, 5, 6], [2, 2, 2]]) #[r, b, e]
    test_inputs = np.array([[0, 0], [1, 1], [2, 2]]) #[u, v]

    ekf = EKF_simple(2, 3, 0, 0.01, 0, 0.01, altitude)
    #I wanna test for 10 EKF iterations
    print("Current State: " + str(ekf.getCurrent_State()))
    print("A Posteriori: " + str(ekf.getAprioriCovariance()))
    for i in range(3):
        ekf.compute_iteration(beacon_x, beacon_y, test_inputs[i, 0], test_inputs[i, 1], 0.2, measures[i], t_step)
        print(ekf.getCurrent_State())
