import numpy as np
import scipy.stats as stats
from utils import draw_sphere_marker

np.set_printoptions(precision=10)

class kalmanfilter:
    def __init__(self, param: list):
        self.A = param['A']
        self.B = param['B']
        self.C = param['C']
        self.Q = param['Q']   # sensor noise
        self.R = param['R']   # motion noise
        self.Sigma = np.eye(3)
        
    def KalmanFilter(self, mu: list, z: list, u: list):

        #prediction step    
        mu_bar = self.A @ mu + self.B @ u
        Sigma_bar = self.A @ self.Sigma @ self.A.T + self.R

        #correction step
        K = Sigma_bar @ self.C.T @ np.linalg.inv(self.C @ Sigma_bar @ self.C.T + self.Q)
        mu_new = mu_bar + K @ (z - self.C @ mu_bar)
        Sigma_new = (np.eye(Sigma_bar.shape[0]) - K @ self.C) @ Sigma_bar

        self.Sigma = Sigma_new   # update sigma
        
        return mu_new, Sigma_new


class particlefilter:
    def __init__(self, param: list):
        self.sample_times = param["Sample_time"]
        self.init_cov = param["Sample_cov"]
        self.sensor_cov = param["Q"]  # sensor noise
        self.R = param["R"]   # motion noise
        self.A = param['A']
        self.B = param['B']
        self.init_state = [0.0, 5.5, -1.57]
        # self.init_state = [2.0, 5.5, -1.57]
        self.particles = np.random.multivariate_normal(self.init_state, self.init_cov, self.sample_times)
        self.W = np.ones(self.sample_times) / self.sample_times
        self.count = 0
        self.particles_list = []

    def PF_update(self, u, z, draw=False):
        self.count += 1

        # predict
        for i in range(self.sample_times):
            self.particles[i] = self.A @ self.particles[i] + self.B @ u
            temp_R = self.R.copy()

            temp_R[0][0] = self.R[0][0] * np.abs(u[0]) + 1e-5
            temp_R[1][1] = self.R[1][1] * np.abs(u[1]) + 1e-5
            temp_R[2][2] = self.R[2][2] * np.abs(u[2]) + 1e-5
            
            self.particles[i] += np.random.multivariate_normal([0, 0, 0], temp_R)

        # update
        weights_sum = 0
        for i in range(self.sample_times):
            p_z = stats.multivariate_normal.pdf(z, self.particles[i], self.sensor_cov)
            self.W[i] *= p_z
            weights_sum += self.W[i]
            
        self.W /= weights_sum   # normalize
                
        # low_variance_resample
        N = self.sample_times
        X_resampled = np.zeros_like(self.particles)
        cum_sum = np.cumsum(self.W)
        step = 1.0 / N
        position = np.random.uniform(0, step)
        i, count = 0, 0
        for m in range(N):
            while position > cum_sum[i]:
                i += 1
            X_resampled[m] = self.particles[i]
            position += step
            
        self.particles = X_resampled
        self.W = np.ones(N) / N
        
        # draw sampling particles
        if (self.count - 1) % 80 == 0:
            self.particles_list.append(self.particles)
            
            if draw:
                for particle in self.particles:
                    draw_sphere_marker((particle[0], particle[1], 0.1), 0.1, (0, 1, 0, 1))
        
        # estimate
        return np.average(self.particles, weights=self.W, axis=0), self.particles_list
    
import numpy as np

# Define the motion model
def motion_model(x, u, dt, noise):
    # x: state vector [x, y, theta]
    # u: control vector [vx, vy, w]
    # dt: time step
    # returns: next state vector
    
    x_new = np.zeros(3)
    x_new[0] += x[0] + dt * u[0]
    x_new[1] += x[1] + dt * u[1]
    x_new[2] += x[2] + dt * u[2]
    mean = [0, 0, 0]
    covariance = np.diag([0.1, 0.1, 0.1])  # Example covariance matrix
    motion_noise = np.random.multivariate_normal(mean, covariance)
    x_new += motion_noise
    
    return x_new

# Define the sensor model
def sensor_model(z, noise):
    # x: state vector [x, y, theta]
    # sigma: standard deviation of the sensor noise
    # returns: noisy measurement of the state vector
    
    # sensor_noise = np.random.multivariate_normal([1.2, 4.5, -1.34], noise)
    sensor_noise = np.random.multivariate_normal([0.1, 0.1, 0.1], noise)
    z += sensor_noise
    
    return z
