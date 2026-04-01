import pybullet as p
import pybullet_data as pd
import time
import math

# --- 1. 环境初始化 ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.8)

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.resetDebugVisualizerCamera(1.5, 45, -30, [0.5, 0, 0.65])

# 加载地面和桌子
p.loadURDF("plane.urdf")
table_pos = [0.5, 0, 0]
p.loadURDF("table/table.urdf", table_pos, useFixedBase=True)

# 加载 Panda 机械臂
panda_pos = [0.5, 0, 0.625]
pandaId = p.loadURDF("franka_panda/panda.urdf", panda_pos, useFixedBase=True)

endEffector = 11


# ================= 任务1：空间坐标系画圆 =================
def draw_circle_cartesian():

    center = [0.5, 0, 0.85]
    radius = 0.15
    steps = 120

    orientation = p.getQuaternionFromEuler([math.pi, 0, 0])

    prev = None

    for i in range(steps):

        angle = 2 * math.pi * i / steps

        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        z = center[2]

        pos = [x, y, z]

        joint_poses = p.calculateInverseKinematics(
            pandaId,
            endEffector,
            pos,
            orientation
        )

        for j in range(7):
            p.setJointMotorControl2(
                pandaId,
                j,
                p.POSITION_CONTROL,
                joint_poses[j],
                force=500
            )

        # 画轨迹
        if prev is not None:
            p.addUserDebugLine(prev, pos, [1, 0, 0], 2)

        prev = pos

        p.stepSimulation()
        time.sleep(1 / 240)

    print("Circle finished")


# ================= 任务2：关节坐标系直线 =================
def draw_line_joint():

    states = p.getJointStates(pandaId, list(range(7)))
    start = [s[0] for s in states]

    target = start.copy()

    target[0] += math.pi / 4
    target[1] -= math.pi / 6
    target[2] += math.pi / 8

    steps = 100

    for i in range(steps):

        current = []

        for j in range(7):
            val = start[j] + (target[j] - start[j]) * i / steps
            current.append(val)

        for j in range(7):
            p.setJointMotorControl2(
                pandaId,
                j,
                p.POSITION_CONTROL,
                current[j],
                force=500
            )

        p.stepSimulation()
        time.sleep(1 / 240)

    print("Joint line finished")


# ================= 执行实验 =================

time.sleep(1)

draw_circle_cartesian()

time.sleep(2)

draw_line_joint()

time.sleep(2)


# ================= 原来的滑块控制系统 =================

mode_toggle = p.addUserDebugParameter("RUN IK (Checked) / RUN JOINT (Unchecked)", 1, 0, 0)

p.addUserDebugText("--- CARTESIAN SETTINGS ---", [1.2, 0.5, 1.2], [0,0,1], 1)

ctrl_x = p.addUserDebugParameter("Target_X", 0.3, 0.8, 0.6)
ctrl_y = p.addUserDebugParameter("Target_Y", -0.4, 0.4, 0.0)
ctrl_z = p.addUserDebugParameter("Target_Z", 0.65, 1.2, 0.8)

p.addUserDebugText("--- JOINT SETTINGS ---", [1.2, -0.5, 1.2], [0,0.5,0], 1)

joint_params = []
joint_names = ["J0_Base","J1_Shoulder","J2_Arm","J3_Elbow","J4_Forearm","J5_Wrist","J6_Flange"]

joint_limits = [
(-2.89,2.89),
(-1.76,1.76),
(-2.89,2.89),
(-3.07,-0.06),
(-2.89,2.89),
(-0.01,3.75),
(-2.89,2.89)
]

for i in range(7):
    joint_params.append(
        p.addUserDebugParameter(
            joint_names[i],
            joint_limits[i][0],
            joint_limits[i][1],
            0.0
        )
    )

info_id = -1


# ================= 主循环 =================

try:

    while True:

        run_ik = p.readUserDebugParameter(mode_toggle)

        if run_ik > 0.5:

            tx = p.readUserDebugParameter(ctrl_x)
            ty = p.readUserDebugParameter(ctrl_y)
            tz = p.readUserDebugParameter(ctrl_z)

            joint_poses = p.calculateInverseKinematics(
                pandaId,
                endEffector,
                [tx, ty, tz],
                p.getQuaternionFromEuler([math.pi, 0, 0])
            )

            for i in range(7):
                p.setJointMotorControl2(
                    pandaId,
                    i,
                    p.POSITION_CONTROL,
                    joint_poses[i],
                    force=500
                )

            mode_str = "CURRENT MODE: CARTESIAN (IK)"

        else:

            for i in range(7):

                target_val = p.readUserDebugParameter(joint_params[i])

                p.setJointMotorControl2(
                    pandaId,
                    i,
                    p.POSITION_CONTROL,
                    target_val,
                    force=500
                )

            mode_str = "CURRENT MODE: JOINT SPACE"


        ee_state = p.getLinkState(pandaId, endEffector)
        curr_p = ee_state[0]

        curr_j = [p.getJointState(pandaId, i)[0] for i in range(7)]

        if info_id != -1:
            p.removeUserDebugItem(info_id)

        display_text = f"{mode_str}\n"
        display_text += f"End-Effector [X,Y,Z]: [{curr_p[0]:.2f}, {curr_p[1]:.2f}, {curr_p[2]:.2f}]\n"
        display_text += "-"*30 + "\n"
        display_text += "Real-time Joints (rad):\n" + "\n".join([f"J{i}: {curr_j[i]:.2f}" for i in range(7)])

        info_id = p.addUserDebugText(display_text, [0.4,-0.8,0.8],[0,0,0],1.2)

        p.stepSimulation()
        time.sleep(1./120.)

except Exception as e:

    print(f"Error occurred: {e}")

finally:

    p.disconnect()