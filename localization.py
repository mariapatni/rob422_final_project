# import numpy as np
# from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker, draw_line
# from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
# from pybullet_tools.pr2_utils import PR2_GROUPS
# import time


# class State:
#     def __init__(self, ):
#         self.position = 
#         self.orientation = 
#         self.

# def kalman_filter(mu_t_prev, sigma_t_prev, mu_t, z_t):
#     """
#     variables:
#     - mu_t_prev: prev state estimate 
#     - sigma_t_prev: prev error covar
#     - A_t: state transition matrix
#     - B_t: control matrix
#     - C_t: measurement matrix
#     - R_t: process noise covar
#     - Q_t: measurement anoise covar
#     - z_t: sensor measurement
#     - eps_t: noise
#     - delta_t: sensor noise

#     return values:
#     - mu_t: updated state estimate
#     - sigma_t: updated error covar
#     """

#     # introduce slight sensor noise
#     # eps_t # ??? noise
#     # delta_t # sensor noise 
#     # # initial state 
#     # # initial state error covar


#     # prediction
#     mu_t_avg = A_t @ mu_t_prev + Bt * mu_t
#     sigma_t_avg = A_t @ (sigma_t_prev @ A_t.T) + R_t
    
#     # correction
#     kalman_gain = np.linalg.inv(C_t @ (sigma_t_avg @ C_t.T) + Q_t)
#     K_t = sigma_T @ (C_t.T @ kalman_gain)

#     sensor_exp = C_t @ mu_t_avg
#     mu_t = mu_t_avg + K_t @ (z_t - sensor_exp) # z_t is sensor measurement

#     shape = (K_t @ C_t).shape # identity mat
#     sigma_t = (np.identity(shape[0])) @ sigma_t_avg # might need to check dimensions for I

#     return mu_t, sigma_t
