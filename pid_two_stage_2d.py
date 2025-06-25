
import pybullet as p
import pybullet_data
import time
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load ground and set camera
p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(cameraDistance=4,
                             cameraYaw=45,
                             cameraPitch=-30,
                             cameraTargetPosition=[0, 0, 0])

# Create cube (red)
cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1], rgbaColor=[1, 0, 0, 1])
cube = p.createMultiBody(baseMass=1,
                         baseCollisionShapeIndex=cube_col,
                         baseVisualShapeIndex=cube_vis,
                         basePosition=[0, 0, 0.1])

# Scattered target positions (X, Y)
target_positions = [(2.0, 1.0), (-1.5, -0.5), (1.0, -1.5)]
target_markers = []
for tx, ty in target_positions:
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[0, 1, 0, 1])
    marker = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[tx, ty, 0.1])
    target_markers.append(marker)

# Sliders
start_slider = p.addUserDebugParameter("Start (0=Stop, 1=Start)", 0, 1, 0)
exit_slider = p.addUserDebugParameter("Exit (0=Wait, 1=Exit)", 0, 1, 0)

# Two PID stages: overshoot then tuned
pid_profiles = [(400, 0, 10, "Initial (overshoot)"),
                (200, 0, 40, "Tuned (minimal overshoot)")]

positions = []
times = []
setpoints_x = []
setpoints_y = []
stages = []

# Wait for start
print("🎚️ Adjust sliders and set 'Start' to 1 to begin...")
while p.readUserDebugParameter(start_slider) < 1:
    p.stepSimulation()
    time.sleep(0.01)

print("🚀 Starting two-stage PID demonstration...")

dt = 1. / 240.
sim_time = 0

for Kp, Ki, Kd, label in pid_profiles:
    print(f"\n▶ Running: {label}  (Kp={Kp}, Ki={Ki}, Kd={Kd})")
    for tx, ty in target_positions:
        integral_x = integral_y = 0
        prev_error_x = prev_error_y = 0

        while True:
            pos, _ = p.getBasePositionAndOrientation(cube)
            x, y = pos[0], pos[1]

            error_x = tx - x
            error_y = ty - y
            integral_x += error_x * dt
            integral_y += error_y * dt
            deriv_x = (error_x - prev_error_x) / dt
            deriv_y = (error_y - prev_error_y) / dt

            force_x = Kp * error_x + Ki * integral_x + Kd * deriv_x
            force_y = Kp * error_y + Ki * integral_y + Kd * deriv_y
            prev_error_x = error_x
            prev_error_y = error_y

            p.applyExternalForce(cube, -1, [force_x, force_y, 0], pos, p.WORLD_FRAME)

            # Log
            positions.append((x, y))
            times.append(sim_time)
            setpoints_x.append(tx)
            setpoints_y.append(ty)
            stages.append(label)

            p.stepSimulation()
            time.sleep(dt)
            sim_time += dt

            if abs(error_x) < 0.05 and abs(error_y) < 0.05 and abs(deriv_x) < 0.1 and abs(deriv_y) < 0.1:
                print(f"✅ Reached target ({tx:.1f}, {ty:.1f}) at t={sim_time:.2f}s")
                time.sleep(0.5)
                break

print("\n✅ Demonstration complete. Set 'Exit' to 1 to plot and close.")
while p.readUserDebugParameter(exit_slider) < 1:
    p.stepSimulation()
    time.sleep(0.01)

p.disconnect()

# Plot result
plt.figure(figsize=(10, 6))
for label in set(set(stages)):
    x_vals = [x for (x, s) in zip(positions, stages) if s == label]
    plt.plot([x[0] for x in x_vals], [x[1] for x in x_vals], label=label)

plt.scatter([x for x, y in target_positions], [y for x, y in target_positions], c='green', marker='o', label='Targets')
plt.title("2D PID Motion with Overshoot vs Tuned Response")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
