import numpy as np

A = np.eye(3)  # Define your state transition matrix
B = np.eye(3)  # Define your control matrix
C = np.eye(3)  # Define your measurement matrix
Q = np.eye(3) * 0.1  # Define your sensor noise covariance
R = np.eye(3) * 0.1  # Define your motion noise covariance 