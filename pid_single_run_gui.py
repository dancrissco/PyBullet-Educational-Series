import pybullet as p
import pybullet_data
import time
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

target_x = 2.0
target_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[0, 1, 0, 1])
p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_vis, basePosition=[target_x, 0, 0.1])

cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1], rgbaColor=[1, 0, 0, 1])
cube = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=cube_col, baseVisualShapeIndex=cube_vis, basePosition=[0, 0, 0.1])

kp_slider = p.addUserDebugParameter("Kp", 0, 500, 200)
ki_slider = p.addUserDebugParameter("Ki", 0, 50, 0)
kd_slider = p.addUserDebugParameter("Kd", 0, 100, 20)
start_slider = p.addUserDebugParameter("Start (0=Stop, 1=Start)", 0, 1, 0)
exit_slider = p.addUserDebugParameter("Exit (0=Wait, 1=Exit)", 0, 1, 0)

print("🎚️ Adjust PID gains and set 'Start' to 1 to begin...")
while p.readUserDebugParameter(start_slider) < 1:
    p.stepSimulation()
    time.sleep(0.01)

print("🚀 Starting simulation...")

integral = 0
prev_error = 0
dt = 1. / 240.
positions, times, forces, errors = [], [], [], []

for i in range(1500):
    Kp = p.readUserDebugParameter(kp_slider)
    Ki = p.readUserDebugParameter(ki_slider)
    Kd = p.readUserDebugParameter(kd_slider)

    cube_pos, _ = p.getBasePositionAndOrientation(cube)
    x = cube_pos[0]

    error = target_x - x
    integral += error * dt
    derivative = (error - prev_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    prev_error = error

    p.applyExternalForce(cube, -1, [output, 0, 0], cube_pos, p.WORLD_FRAME)

    positions.append(x)
    times.append(i * dt)
    forces.append(output)
    errors.append(error)

    p.stepSimulation()
    time.sleep(1. / 120.)

print("\n✅ Simulation complete. Set 'Exit' slider to 1 to close GUI and show plots.")
while p.readUserDebugParameter(exit_slider) < 1:
    p.stepSimulation()
    time.sleep(0.01)

p.disconnect()

plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(times, positions, label="Cube X Position")
plt.axhline(target_x, color='g', linestyle='--', label="Target")
plt.title("Position Over Time")
plt.xlabel("Time (s)")
plt.ylabel("X Position (m)")
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(times, forces, label="PID Output Force")
plt.title("PID Force Output Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.legend()
plt.tight_layout()
plt.show()