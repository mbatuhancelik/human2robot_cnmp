import pybullet_data

import numpy as np

import os
import time
import math


class Torobo:
    def __init__(self, p):

        self.plane_id = None
        self.robot_id = None
        self.duck = None
        self._p = p
        self.hz = 240.
        self._p.setTimeStep(1.0 / self.hz)
        self._p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40,
                                          cameraTargetPosition=[0.55, -0.35, 0.2])
        self._last_time = 0.0
        self._timestep = self._p.getPhysicsEngineParameters()["fixedTimeStep"]
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_SHADOWS, 0)

        self.link = 33

        self.unit_step_length = 0.01  # 1cm

        self.robot_id = self._p.loadURDF('torobo_robot/torobo.urdf', globalScaling=1.5)
        self.plane_id = self._p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane_transparent.urdf"),
                                        basePosition=[0, 0, 0])

        # for link in range(-1, self.p.getNumJoints(self.robot)):
        #     self.p.changeVisualShape(self.robot, link, rgbaColor=[0, 0, 0, 0])

        # Torobo robot configurations
        self.init_robot()
        self.robotJointsDOF = None
        self.robotJointsDamping = None
        self.robotJointsForces = None

        # Control configurations
        self.jointsForCtrl = None  # move only right arm joints [22, 23, 24, 25, 26, 27, 28, 32, 33]
        self.jointsMaxForceCtrl = None
        self._p.setGravity(0, 0, -10)

        self.jointConfigurations()

    def initialize(self):
        self.init_robot()

    def jointConfigurations(self):
        self.robotJointsDOF = [4, 5, 8, 9, 10, 11, 12, 13, 14, 18, 19, 22, 23, 24, 25, 26, 27, 28, 32, 33, 36, 37]
        numJoints = self._p.getNumJoints(self.robot_id)
        jointsMaxForce = []
        # jointsMaxVelocity = []

        for jointIndex in range(numJoints):
            jointInfo = self._p.getJointInfo(self.robot_id, jointIndex)
            jointType = jointInfo[2]
            if jointType != self._p.JOINT_FIXED:  # Exclude fixed joints
                jointsMaxForce.append(jointInfo[10])
                # jointsMaxVelocity.append(jointInfo[11])

        # Joint damping coefficients.
        jointsDamping = [100.0 for _ in range(22)]  # Use large values for the joints that we don't want to move
        jointsDamping[11:18] = 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
        self.robotJointsDamping = jointsDamping
        self.robotJointsForces = jointsMaxForce
        self.jointsForCtrl = self.robotJointsDOF[11:18]
        self.jointsMaxForceCtrl = self.robotJointsForces[11:18]
        # jointsMaxVelCtrl = jointsMaxVelocity[11:20]

    def init_robot(self):
        # self.p.resetJointState(self.robot_id, 5, math.radians(20))  # torso joint 2
        self._p.resetJointState(self.robot_id, 9, math.radians(90))  # left arm joint 2
        self._p.resetJointState(self.robot_id, 13, math.radians(-90))

        self._p.resetJointState(self.robot_id, 22, math.radians(54))  # right arm joint 1
        self._p.resetJointState(self.robot_id, 23, math.radians(42))  # right arm joint 2
        self._p.resetJointState(self.robot_id, 24, math.radians(58))  # right arm joint 3
        self._p.resetJointState(self.robot_id, 25, math.radians(84))  # right arm joint 4
        self._p.resetJointState(self.robot_id, 26, math.radians(-32))  # right arm joint 5
        self._p.resetJointState(self.robot_id, 27, math.radians(21))  # right arm joint 6
        self._p.resetJointState(self.robot_id, 28, math.radians(-180))  # right arm joint 7

    def get_link_pos_ori(self, link_index):
        gripper_information = self._p.getLinkState(self.robot_id, link_index)
        return list(gripper_information[0]), list(self._p.getEulerFromQuaternion(gripper_information[1]))

    def get_base_pos_ori(self, obj_id):
        info = self._p.getBasePositionAndOrientation(obj_id)
        return list(info[0]), list(self._p.getEulerFromQuaternion(info[1]))

    def get_base_velocity_linear(self, obj_id):
        info = self._p.getBaseVelocity(obj_id)
        return list(info[0])

    def run(self, bodyUniqueId, endEffectorLinkIndex, pos):

        ori = self._p.getQuaternionFromEuler([math.radians(180), math.radians(0), math.radians(-180.)])
        conf = self._p.calculateInverseKinematics(bodyUniqueId=bodyUniqueId, endEffectorLinkIndex=endEffectorLinkIndex,
                                                 targetPosition=pos, targetOrientation=ori,
                                                 jointDamping=self.robotJointsDamping)

        confCtrl = conf[11:20]

        self._p.setJointMotorControlArray(bodyUniqueId=bodyUniqueId, jointIndices=self.jointsForCtrl,
                                         controlMode=self._p.POSITION_CONTROL, targetPositions=confCtrl,
                                         forces=self.jointsMaxForceCtrl)

        # Alternatively, you can set the joint's angular limits to zero
        self._p.setJointMotorControl2(self.robot_id, jointIndex=4, controlMode=self._p.POSITION_CONTROL,
                                     targetPosition=0, targetVelocity=0, positionGain=0, velocityGain=1)
        self._p.setJointMotorControl2(self.robot_id, jointIndex=5, controlMode=self._p.POSITION_CONTROL,
                                     targetPosition=0, targetVelocity=0, positionGain=0, velocityGain=1)

        iterations = 0
        while self.wait(pos) and iterations < 200:
            iterations += 1
            self._p.stepSimulation()
            time.sleep(2.0 / self.hz)
    def set_joint_position(self, position, velocity=None, t=None, sleep=False, traj=False):
        assert len(self.jointsForCtrl) > 0
        if traj:
            assert (t is not None)
            N = int(t * 240)
            current_position = self.get_joint_position()[:-2]
            trajectory = np.linspace(current_position, position, N)
            running_force_feedback = np.zeros(8, dtype=np.float32)
            for i, t_i in enumerate(trajectory):
                self._p.setJointMotorControlArray(
                    bodyUniqueId=self.robot_id,
                    jointIndices=self.jointsForCtrl,
                    controlMode=self._p.POSITION_CONTROL,
                    targetPositions=t_i,
                    forces=self.jointsMaxForceCtrl)
                self._p.stepSimulation()
                # print("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f" % tuple(self.get_joint_forces()))
                force_feedback = np.array(self.get_joint_forces())
                running_force_feedback = 0.9 * running_force_feedback + 0.1 * force_feedback
                if running_force_feedback[5] < self.force_stop_threshold:
                    # print("="*100)
                    for j in range(N//20):
                        self._p.setJointMotorControlArray(
                            bodyUniqueId=self.id,
                            jointIndices=self.joints[:-2],
                            controlMode=self._p.POSITION_CONTROL,
                            targetPositions=trajectory[i-j],
                            forces=self.jointsMaxForceCtrl)
                        self._p.stepSimulation()
                        if sleep:
                            time.sleep(self._timestep)
                        force_feedback = np.array(self.get_joint_forces())
                    break
                if sleep:
                    time.sleep(self._timestep)

        else:
            if velocity is not None:
                self._p.setJointMotorControlArray(
                    bodyUniqueId=self.id,
                    jointIndices=self.joints[:-2],
                    controlMode=self._p.POSITION_CONTROL,
                    targetPositions=position,
                    targetVelocities=velocity,
                    forces=self.jointsMaxForceCtrl)
            else:
                range = len(position)
                self._p.setJointMotorControlArray(
                    bodyUniqueId=self.robot_id,
                    jointIndices=self.jointsForCtrl[:range],
                    controlMode=self._p.POSITION_CONTROL,
                    targetPositions=position,
                    forces=self.jointsMaxForceCtrl[:range])
            self._waitsleep(t, sleep)
    def move_in_cartesian(self, position, orientation = None, t=1.0, sleep=False, ignore_force=False):
        N = int(t * 240)
        if orientation is None:
            orientation = self._p.getQuaternionFromEuler([np.pi, 0, -np.pi])
        current_position, current_orientation = self.get_link_pos_ori(self.link)

        position_traj = np.linspace(current_position, position, N+1)[1:]
        # orientation_traj = np.linspace(current_orientation, orientation, N+1)[1:]
        # running_force_feedback = np.zeros(8, dtype=np.float32)
        joint_history = np.zeros((position_traj.shape[0], len(self.jointsForCtrl)))
        pos_history = np.zeros((position_traj.shape[0], 3))
        for i, p_i in enumerate(position_traj):
            target_joints = self._p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.link,
                targetPosition=p_i,
                targetOrientation=orientation,
                jointDamping=self.robotJointsDamping
            )
            joint_history[i] = target_joints[11:18]
            self.set_joint_position(target_joints[11:18], t=1/240, sleep=sleep)
            pos_history[i] = self.get_link_pos_ori(self.link)[0]
            # force_feedback = np.array(self.get_joint_forces())
            # running_force_feedback = 0.9 * running_force_feedback + 0.1 * force_feedback
            # if not ignore_force:
            #     if running_force_feedback[5] < self.force_stop_threshold:
            #         # print("="*100)
            #         for j in range(N//20):
            #             target_joints = self.p.calculateInverseKinematics(
            #                 bodyUniqueId=self.id,
            #                 endEffectorLinkIndex=self.ik_idx,
            #                 targetPosition=position_traj[i-j],
            #                 targetOrientation=orientation)
            #             self.set_joint_position(target_joints[:-2], t=1/240, sleep=sleep)
            #             force_feedback = np.array(self.get_joint_forces())
            #         break
        return joint_history, pos_history
    def set_cartesian_position(self, position, orientation=None, t=None, sleep=False, traj=False):
        if orientation is None:
            orientation = self._p.getQuaternionFromEuler([np.pi, 0, -np.pi])
        target_joints = self._p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.link,
                targetPosition=position,
                targetOrientation=orientation,
                jointDamping=self.robotJointsDamping
            )
        self.set_joint_position(target_joints[11:18], t=1/240, sleep=sleep)
    def _waitsleep(self, t, sleep=False):
        if t is not None:
            iters = int(t*240)
            for _ in range(iters):
                self._p.stepSimulation()
                if sleep:
                    while time.time() - self._last_time < self._timestep:
                        time.sleep(0.0001)
                    self._last_time = time.time()
    def wait(self, goal_pos):
        pos, ori = self.get_link_pos_ori(self.link)

        distance = np.linalg.norm(np.array(pos) - np.array(goal_pos))
        if distance < 0.01:
            return False

        return True

    def reset(self):
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 0)
        self.init_robot()
        self._p.resetBasePositionAndOrientation(self.duck, self.duck_init_pos, self.duck_init_ori)
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)

    def episode(self, action):

        start_img = self.render()
        self.step(action)

        rgb_array = self.render()
        time.sleep(1)

        return start_img, rgb_array

    def close(self):
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 0)
        self.init_robot()
        self._p.resetBasePositionAndOrientation(self.duck, self.duck_init_pos, self.duck_init_ori)
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)

    def render(self, mode='human'):
        view_matrix = self._p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[1.4, 0, 0.05],
                                                               distance=1.5,
                                                               yaw=90,
                                                               pitch=-90,
                                                               roll=0,
                                                               upAxisIndex=2)

        proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
                                                        aspect=float(960) / 720,
                                                        nearVal=0.1,
                                                        farVal=100.0)

        # Returns (width, height, rgbPixels, depthPixels, segmentationMaskBuffer)
        # rgb pixels -> list of [char RED,char GREEN,char BLUE, char ALPHA] [0..width*height]
        # depthPixels > list of float [0..width*height]
        # segmentationMaskBuffer -> list of int [0..width*height]
        (_, _, px, _, _) = self._p.getCameraImage(width=128,
                                                 height=128,
                                                 viewMatrix=view_matrix,
                                                 projectionMatrix=proj_matrix,
                                                 renderer=self._p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)

        rgb_array = rgb_array[:, :, :3]
        return rgb_array
