import pybullet as p
import pybullet_data
import time
import random  # Import the random module

def setup_simulation(num_obstacles=5, area_size=5, seed=42):
    # Set the random seed for reproducibility
    random.seed(seed)
    
    # Connect to PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load the plane and PR2 robot
    plane_id = p.loadURDF("plane.urdf")
    pr2_id = p.loadURDF("models/drake/pr2_description/urdf/pr2_simplified.urdf", [0, 0, 0.5]) 

    # Add random obstacles
    obstacle_ids = []
    for _ in range(num_obstacles):
        x = random.uniform(-area_size, area_size)
        y = random.uniform(-area_size, area_size)
        z = 0.5  # Assuming the obstacle is 1 unit tall
        obstacle_id = p.loadURDF("cube.urdf", [x, y, z], globalScaling=0.5)
        obstacle_ids.append(obstacle_id)

    # Set gravity
    p.setGravity(0, 0, -9.81)

    return pr2_id, obstacle_ids

def simulate(pr2_id, steps=1000):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./24000000.)

    p.disconnect()

if __name__ == "__main__":
    pr2_id, obstacle_ids = setup_simulation()
    simulate(pr2_id) 