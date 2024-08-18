import pybullet as p
import numpy as np
import pybullet_data
import time

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
            "rpy": np.zeros(3),
            "quat": np.array(start_orientation)
        }

    def get_drone_mass(self):
        mass = p.getDynamicsInfo(self.drone_id, -1)[0]  # -1 gets the base link mass
        return mass

    def get_drones_kinematic_info(self):
        pos, quat = p.getBasePositionAndOrientation(self.drone_id)
        vel, ang_vel = p.getBaseVelocity(self.drone_id)
        rpy = p.getEulerFromQuaternion(quat)

        self.drones_kinematic_info = {
            "pos": np.array(pos),
            "vel": np.array(vel),
            "rpy": np.array(rpy),
            "quat": np.array(quat)
        }

        return self.drones_kinematic_info

    def update_drone_state(self, target_pos, target_orientation):
        # Update the drone's position and orientation
        p.resetBasePositionAndOrientation(self.drone_id, target_pos, target_orientation)

    def close(self):
        p.removeBody(self.drone_id)  # Remove the drone from the simulation
        p.disconnect()  # Disconnect the PyBullet environment

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
    sim_env = DroneBltEnv(drone_urdf_path="assets/x_drone.urdf", start_pos=[0, 0, 0.1])

    # Load assets:
    p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])  # Load the plane URDF to act as the ground

    # Define the movement speed and rotation speed
    move_speed = 0.05
    rotation_speed = 0.1  # Rotation speed in radians

    while p.isConnected():
        p.stepSimulation()

        # Get the current drone state
        current_state = sim_env.get_drones_kinematic_info()
        current_pos = current_state['pos']
        current_rpy = current_state['rpy']

        # Get pressed keys
        key_codes = get_key_pressed()

        # Update position based on key press
        # Fuck it in QWERTY 
        for key in key_codes:
            if p.B3G_UP_ARROW in key_codes:
                current_pos[2] += move_speed  # Up
            if p.B3G_DOWN_ARROW in key_codes:
                current_pos[2] -= move_speed  # Down
            if ord('w') in key_codes:
                current_pos[1] += move_speed  # Forward
            if ord('s') in key_codes:
                current_pos[1] -= move_speed  # Backward
            if ord('a') in key_codes:
                current_pos[0] -= move_speed  # Left
            if ord('d') in key_codes:
                current_pos[0] += move_speed  # Right
            if ord('q') in key_codes:
                current_rpy[2] += rotation_speed  # Counterclockwise rotation (yaw increase)
            if ord('e') in key_codes:
                current_rpy[2] -= rotation_speed  # Clockwise rotation (yaw decrease)

        # Convert the updated roll, pitch, yaw to a quaternion
        target_orientation = p.getQuaternionFromEuler(current_rpy)

        # Update drone position and orientation
        sim_env.update_drone_state(current_pos, target_orientation)

        time.sleep(0.01)  # Small delay for simulation stability

    # Close the environment
    sim_env.close()