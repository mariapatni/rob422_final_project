import numpy as np
from utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, get_joint_info, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
# from localization import kalman_filter
from path import *
import time

def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('pr2table.json')
    # get the index for PR2
    PR2 = robots['pr2']

    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    # define active DoFs
    joint_names =('l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint')
    joint_idx = [joint_from_name(robots['pr2'], jn) for jn in joint_names]

    # parse active DoF joint limits
    #joint_limits = {joint_names[i] : (get_joint_info(robots['pr2'], joint_idx[i]).jointLowerLimit, get_joint_info(robots['pr2'], joint_idx[i]).jointUpperLimit) for i in range(len(joint_idx))}

    start_config = tuple(get_joint_positions(robots['pr2'], base_joints))
    # goal_config = 

    
    # path = np.array([section1_start, section1_end, section2_start, section2_end, section3_start, section3_end, section4_start,  \
    #             section4_end, section5_start, section5_end, section6_start, section6_end, section7_start, section7_end, \
    #             section8_start, section8_end, section9_start, section9_end, section10_start, section10_end, section11_start, \
    #             section11_end, section12_start, section12_end])
    
    path = []
    # append waypoints to path
    path.append(section1_start)
    path.append(section1_end)
    path.append(section2_start)
    path.append(section2_end)
    path.append(section3_start)
    path.append(section3_end)
    path.append(section4_start)
    path.append(section4_end)
    path.append(section5_start)
    path.append(section5_end)
    path.append(section6_start)
    path.append(section6_end)
    path.append(section7_start)
    path.append(section7_end)
    path.append(section8_start)
    path.append(section8_end)
    path.append(section9_start)
    path.append(section9_end)
    path.append(section10_start)
    path.append(section10_end)
    path.append(section11_start)
    path.append(section11_end)
    path.append(section12_start)
    path.append(section12_end)
  

    # execute Path
    execute_trajectory(PR2, joint_idx, path, sleep=0.1)
    
    start_time = time.time()
    #############################################
    
    # z_t = np.array([get_current_position(robots['pr2'])])
    # mu_init
    # sigma_init
    # for():
    #     mu_t, sigma_t = kalman_filter(mu_t_prev, sigma_t_prev, A_t, B_t, C_t, R_t, Q_t, z_t)
    
    ##############################################
    
        
    print("run time: ", time.time() - start_time)
    # Execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()