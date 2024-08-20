import pybullet as p
import numpy as np
import pybullet_data
import time

'''
Ressources used for this project: 
    - https://github.com/utiasDSL/gym-pybullet-drones
    - https://github.com/toritamantaro/pybullet-drone-self-study
'''

class DroneBltEnv:
    def __init__(self, drone_urdf_path, start_pos=[0, 0, 0.1], start_orientation=[0, 0, 0, 1], time_step=1./240., max_speed_kmh=20):
        self.drone_id = p.loadURDF(drone_urdf_path, start_pos, start_orientation)
        self.time_step = time_step
        self.max_speed_kmh = max_speed_kmh

        # Store drone properties
        self.drone_properties = {
            "max_speed_kmh": max_speed_kmh,
            "mass": self.get_drone_mass(),
        }

        # Initialize the drone's kinematic information
        self.drones_kinematic_info = {
            "pos": np.array(start_pos),
            "vel": np.zeros(3),
            "quat": np.array(start_orientation)
        }

    def get_drone_mass(self):
        mass = p.getDynamicsInfo(self.drone_id, -1)[0]
        return mass

    def get_drones_kinematic_info(self):
        pos, quat = p.getBasePositionAndOrientation(self.drone_id)
        vel, ang_vel = p.getBaseVelocity(self.drone_id)

        self.drones_kinematic_info = {
            "pos": np.array(pos),
            "vel": np.array(vel),
            "quat": np.array(quat)
        }

        return self.drones_kinematic_info

    def update_drone_state(self, target_pos, target_orientation):
        # Update the drone's position and orientation
        p.resetBasePositionAndOrientation(self.drone_id, target_pos, target_orientation)

    def close(self):
        p.removeBody(self.drone_id)
        p.disconnect()

def get_key_pressed():
    pressed_keys = []
    events = p.getKeyboardEvents()
    key_codes = events.keys()
    for key in key_codes:
        pressed_keys.append(key)
    return pressed_keys

if __name__ == "__main__":
    # Initialize the pybullet environment.
    p.connect(p.GUI)  # Human rendering interface / p.DIRECT otherwise
    p.resetSimulation()
    p.setRealTimeSimulation(0)
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the path to PyBullet's data files

    # Initialize the simulation environment
    sim_env = DroneBltEnv(drone_urdf_path="assets/generic_pdrone.urdf", start_pos=[0, 0, 0.1])

    # Load assets:
    p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])  # Load the plane URDF to act as the ground

    move_speed = 0.05
    rotation_speed = 0.1

    while p.isConnected():
        p.stepSimulation()

        # Get the current drone state
        current_state = sim_env.get_drones_kinematic_info()
        current_pos = current_state['pos']
        current_quat = current_state['quat']

        key_codes = get_key_pressed()

        # Calculate the drone's local frame movement
        forward_vector = np.array([0, move_speed, 0]) # Deal Forward/Backward movement with rot
        right_vector = np.array([move_speed, 0, 0])   # Deal Left/Right  movement with rot

        local_rotation_matrix = np.array(p.getMatrixFromQuaternion(current_quat)).reshape(3, 3)
        
        forward_direction = np.dot(local_rotation_matrix, forward_vector)
        right_direction = np.dot(local_rotation_matrix, right_vector)

        # Update position based on key press
        for key in key_codes:
            if ord('u') in key_codes:
                current_pos[2] += move_speed  # Up
            if ord('d') in key_codes:
                current_pos[2] -= move_speed  # Down
            if p.B3G_UP_ARROW in key_codes:
                current_pos += forward_direction  # Move forward 
            if p.B3G_DOWN_ARROW in key_codes:
                current_pos -= forward_direction  # Move backward 
            if p.B3G_LEFT_ARROW in key_codes:
                current_pos -= right_direction  # Move left 
            if p.B3G_RIGHT_ARROW in key_codes:
                current_pos += right_direction  # Move right 
            # Apply rotation
            if ord('q') in key_codes:
                # Counterclockwise rotation
                rotation_quat = p.getQuaternionFromEuler([0, 0, rotation_speed])
                current_quat = p.multiplyTransforms([0, 0, 0], current_quat, [0, 0, 0], rotation_quat)[1]
            if ord('e') in key_codes:
                # Clockwise rotation
                rotation_quat = p.getQuaternionFromEuler([0, 0, -rotation_speed])
                current_quat = p.multiplyTransforms([0, 0, 0], current_quat, [0, 0, 0], rotation_quat)[1]

        # Update drone position and orientation
        sim_env.update_drone_state(current_pos, current_quat)

        time.sleep(0.01)

    sim_env.close()


