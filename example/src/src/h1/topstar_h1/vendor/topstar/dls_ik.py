import numpy as np
from math import pi, cos, sin
from scipy.spatial.transform import Rotation as R


class IIWAIK:
    def __init__(self):
        # Kuka iiwa 7 R800 DH parameters (in meters and radians)
        # Source: Kuka official documentation
        self.dh_params = [
            # a, alpha, d, theta_offset
            [0.0, -pi / 2, 0.0, 0],  # Joint 1
            [0.0, pi / 2, 0.0, 0],  # Joint 2
            [0.0, -pi / 2, 0.300, 0],  # Joint 3
            [0.0, pi / 2, 0.0, 0],  # Joint 4
            [0.0, -pi / 2, 0.2447, 0],  # Joint 5
            [0.0, pi / 2, 0.0, 0],  # Joint 6
            [0.0, 0.0, 0.1468, 0]  # Joint 7
        ]

        # Joint limits (radians)
        self.joint_limits = [
            [-150.0, 150.0],  # Joint 1
            [-90.0, 25.0],  # Joint 2
            [-150.0, 170.0],  # Joint 3
            [-103.0, 25.0],  # Joint 4
            [-165.0, 165.0],  # Joint 5
            [-88.0, 25.0],  # Joint 6
            [-170.0, 170.0]  # Joint 7
        ]
        self.joint_limits = np.deg2rad(np.array(self.joint_limits))

        # DLS parameters
        self.lambda_max = 1.0  # Maximum damping factor
        self.lambda_min = 0.001  # Minimum damping factor
        self.sigma_threshold = 0.01  # Singularity threshold
        self.debug = False

    def dh_transform(self, a, alpha, d, theta):
        """Create DH transformation matrix"""
        ct = cos(theta)
        st = sin(theta)
        ca = cos(alpha)
        sa = sin(alpha)

        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles):
        """Compute forward kinematics for given joint angles"""
        T = np.eye(4)

        for i in range(7):
            a, alpha, d, theta_offset = self.dh_params[i]
            theta = joint_angles[i] + theta_offset
            T_i = self.dh_transform(a, alpha, d, theta)
            T = T @ T_i

        return T

    def compute_jacobian(self, joint_angles):
        """Compute the geometric Jacobian matrix"""
        # Current transformation matrices for each joint
        T = [np.eye(4)] * 8  # Base to each joint (0-7)
        T[0] = np.eye(4)

        # Compute forward kinematics up to each joint
        for i in range(7):
            a, alpha, d, theta_offset = self.dh_params[i]
            theta = joint_angles[i] + theta_offset
            T_i = self.dh_transform(a, alpha, d, theta)
            T[i + 1] = T[i] @ T_i

        # End-effector position
        p_ee = T[7][:3, 3]

        # Jacobian matrix (6x7)
        J = np.zeros((6, 7))

        for i in range(7):
            # Joint axis (z-axis of previous frame)
            if i == 0:
                z_i = np.array([0, 0, 1])
            else:
                z_i = T[i][:3, 2]

            # Position vector from joint i to end-effector
            p_i = T[i][:3, 3]
            r_i = p_ee - p_i

            # Linear velocity part (cross product)
            J[:3, i] = np.cross(z_i, r_i)

            # Angular velocity part
            J[3:, i] = z_i

        return J

    def compute_pose_error(self, T_current, T_target):
        """Compute pose error (position + orientation)"""
        # Position error
        p_error = T_target[:3, 3] - T_current[:3, 3]

        # Orientation error using axis-angle representation
        R_current = T_current[:3, :3]
        R_target = T_target[:3, :3]
        R_error = R_target @ R_current.T

        # Convert rotation matrix to axis-angle
        try:
            r = R.from_matrix(R_error)
            axis_angle = r.as_rotvec()
        except:
            axis_angle = np.zeros(3)

        # Combine errors
        error = np.concatenate([p_error, axis_angle])

        return error

    def adaptive_damping(self, J, current_error_norm):
        """Compute adaptive damping factor based on Jacobian condition and error"""
        # Compute singular values
        try:
            U, s, Vt = np.linalg.svd(J @ J.T)
            min_singular = np.min(s)
        except:
            min_singular = 0

        # Base damping on singularity proximity
        if min_singular < self.sigma_threshold:
            lambda_damp = self.lambda_max * (1 - min_singular / self.sigma_threshold)
        else:
            lambda_damp = self.lambda_min

        # Increase damping if error is large (for stability)
        error_factor = min(1.0, current_error_norm / 0.1)  # Normalize error
        lambda_damp = lambda_damp + (self.lambda_max - lambda_damp) * error_factor

        return lambda_damp

    def inverse_kinematics(self, target_pose, initial_angles=None,
                               max_iterations=100, tolerance=1e-5):
        """Damped Least Squares IK with adaptive damping"""

        # Initialize joint angles
        if initial_angles is None:
            joint_angles = np.zeros(7)
        else:
            joint_angles = np.array(initial_angles).copy()

        # Store convergence data
        errors = []
        lambdas = []

        for iteration in range(max_iterations):
            # Current transformation
            T_current = self.forward_kinematics(joint_angles)

            # Compute error
            error = self.compute_pose_error(T_current, target_pose)
            error_norm = np.linalg.norm(error)
            errors.append(error_norm)

            # Check convergence
            if error_norm < tolerance:
                self.debug_print(f"Converged after {iteration} iterations with error: {error_norm:.6f}")
                break

            # Compute Jacobian
            J = self.compute_jacobian(joint_angles)

            # Adaptive damping
            lambda_damp = self.adaptive_damping(J, error_norm)
            lambdas.append(lambda_damp)

            # DLS solution: Δθ = Jᵀ(JJᵀ + λ²I)⁻¹e
            JJT = J @ J.T
            damping_matrix = (lambda_damp ** 2) * np.eye(6)

            try:
                # Solve (JJᵀ + λ²I)Δx = e for Δx
                delta_x = np.linalg.solve(JJT + damping_matrix, error)
                # Compute joint update: Δθ = JᵀΔx
                delta_theta = J.T @ delta_x
            except np.linalg.LinAlgError:
                # Use pseudo-inverse if singular
                J_pinv = np.linalg.pinv(J)
                delta_theta = J_pinv @ error

            # Update joint angles
            joint_angles += delta_theta

            # Apply joint limits
            for i in range(7):
                joint_angles[i] = np.clip(joint_angles[i],
                                          self.joint_limits[i, 0],
                                          self.joint_limits[i, 1])

            if iteration % 10 == 0:
                self.debug_print(f"Iteration {iteration}: error = {error_norm:.6f}, lambda = {lambda_damp:.4f}")

        else:
            self.debug_print(f"Max iterations reached. Final error: {error_norm:.6f}")

        if self.debug:
            return joint_angles, errors, lambdas
        return joint_angles, error_norm

    def create_target_pose(self, position, euler_angles):
        """Create target pose from position and Euler angles"""
        T = np.eye(4)
        T[:3, 3] = position
        T[:3, :3] = R.from_euler('xyz', euler_angles).as_matrix()
        return T

    def debug_print(self, _str):
        if self.debug:
            print(_str)
