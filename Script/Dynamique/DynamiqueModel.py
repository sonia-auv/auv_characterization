import numpy as np


class DynamicModel:

    AUV_MASS = 49.5
    AUV_VOLUME = 0.00283496207
    WATER_DENSITY = 1000
    X_DRAG_COEFFICIENT = 0.0
    Y_DRAG_COEFFICIENT = 0.0
    Z_DRAG_COEFFICIENT = 0.0
    ROLL_DRAG_COEFFICIENT = 0.0
    PITCH_DRAG_COEFFICIENT = 0.0
    YAW_DRAG_COEFFICIENT = 0.0
    BUOYANCY_CENTER = np.array([0.0, 0.0, 0.0])
    X_SURFACE = 0.73 * 0.17
    Y_SURFACE = X_SURFACE
    Z_SURFACE = X_SURFACE + Y_SURFACE
    ROLL_SURFACE = 0.0
    PITCH_SURFACE = 0.0
    YAW_SURFACE = 0.40 * 0.17
    GRAVITY = AUV_MASS * -9.81
    BUOYANCY = AUV_VOLUME * WATER_DENSITY

    def __init__(self):
        # M_A
        self.added_mass_matrix = None
        # M_RB
        self.body_mass_inertia_matrix = None
        # M
        self.mass_matrix = None
        # D(v)
        self.damping_matrix = None
        # g(n)
        self.gravity_vector = None
        # T
        self.thrust = None

        self.set_added_mass_matrix()
        self.set_body_mass_inertia_matrix()
        self.set_mass_matrix()

    def set_added_mass_matrix(self):
        add_mass_x = self.AUV_VOLUME * self.WATER_DENSITY
        add_mass_y = self.AUV_VOLUME * self.WATER_DENSITY
        add_mass_z = self.AUV_VOLUME * self.WATER_DENSITY
        add_mass_roll = self.AUV_VOLUME * self.WATER_DENSITY
        add_mass_pitch = self.AUV_VOLUME * self.WATER_DENSITY
        add_mass_yaw = self.AUV_VOLUME * self.WATER_DENSITY
        self.added_mass_matrix = np.array([[add_mass_x, 0.0,        0.0,        0.0,           0.0,            0.0],
                                           [0.0,        add_mass_y, 0.0,        0.0,           0.0,            0.0],
                                           [0.0,        0.0,        add_mass_z, 0.0,           0.0,            0.0],
                                           [0.0,        0.0,        0.0,        add_mass_roll, 0.0,            0.0],
                                           [0.0,        0.0,        0.0,        0.0,           add_mass_pitch, 0.0],
                                           [0.0,        0.0,        0.0,        0.0,           0.0,            add_mass_yaw]])

    def set_body_mass_inertia_matrix(self):
        self.body_mass_inertia_matrix = np.array([[self.AUV_MASS, 0.0,           0.0,           0.0,  0.0,   0.0],
                                                  [0.0,           self.AUV_MASS, 0.0,           0.0,  0.0,   0.0],
                                                  [0.0,           0.0,           self.AUV_MASS, 0.0,  0.0,   0.0],
                                                  [0.0,           0.0,           0.0,           1.99, 0.0,   0.0],
                                                  [0.0,           0.0,           0.0,           0.0,  3.046, 0.0],
                                                  [0.0,           0.0,           0.0,           0.0,  0.0,   1.702]])

    def set_mass_matrix(self):
        self.mass_matrix = self.body_mass_inertia_matrix + self.added_mass_matrix

    def set_damping_matrix(self, speed):

        drag_x = 0.5 * self.WATER_DENSITY * np.power(speed[0], 2) * self.X_SURFACE * self.X_DRAG_COEFFICIENT
        drag_y = 0.5 * self.WATER_DENSITY * np.power(speed[1], 2) * self.Y_SURFACE * self.Y_DRAG_COEFFICIENT
        drag_z = 0.5 * self.WATER_DENSITY * np.power(speed[2], 2) * self.Z_SURFACE * self.Z_DRAG_COEFFICIENT
        drag_roll = 0.5 * self.WATER_DENSITY * np.power(speed[3], 2) * self.ROLL_SURFACE * self.ROLL_DRAG_COEFFICIENT
        drag_pitch = 0.5 * self.WATER_DENSITY * np.power(speed[4], 2) * self.PITCH_SURFACE * self.PITCH_DRAG_COEFFICIENT
        drag_yaw = 0.5 * self.WATER_DENSITY * np.power(speed[5], 2) * self.YAW_SURFACE * self.YAW_DRAG_COEFFICIENT

        self.damping_matrix = np.array([[drag_x, 0.0,    0.0,    0.0,       0.0,        0.0],
                                        [0.0,    drag_y, 0.0,    0.0,       0.0,        0.0],
                                        [0.0,    0.0,    drag_z, 0.0,       0.0,        0.0],
                                        [0.0,    0.0,    0.0,    drag_roll, 0.0,        0.0],
                                        [0.0,    0.0,    0.0,    0.0,       drag_pitch, 0.0],
                                        [0.0,    0.0,    0.0,    0.0,       0.0,        drag_yaw]])

    def set_gravity_vector(self, orientation):
        g_x = (self.BUOYANCY - self.GRAVITY) * np.sin(orientation[1])
        g_y = -(self.BUOYANCY - self.GRAVITY) * np.sin(orientation[0]) * np.cos(orientation[1])
        g_z = -(self.BUOYANCY - self.GRAVITY) * np.cos(orientation[0]) * np.cos(orientation[1])
        g_roll = self.BUOYANCY * np.cos(orientation[1]) * (self.BUOYANCY_CENTER[2] * np.sin(orientation[0]) - self.BUOYANCY_CENTER[1] * np.cos(orientation[0]))
        g_pitch = self.BUOYANCY * (self.BUOYANCY_CENTER[0] * np.cos(orientation[0]) * np.cos(orientation[1])- self.BUOYANCY_CENTER[2] * np.sin(orientation[0]))
        g_yaw = -self.BUOYANCY * (self.BUOYANCY_CENTER[2] * np.sin(orientation[0]) * np.cos(orientation[1]) - self.BUOYANCY_CENTER[1] * np.sin(orientation[0]))
        self.gravity_vector = np.array([[g_x],
                                        [g_y],
                                        [g_z],
                                        [g_roll],
                                        [g_pitch],
                                        [g_yaw]])

    def compute_dynamic_model(self, orientation, speed):
        self.set_damping_matrix(speed)
        self.set_gravity_vector(orientation)
        acc = 0.0
        self.thrust = self.mass_matrix * acc + self.damping_matrix + self.gravity_vector



    





