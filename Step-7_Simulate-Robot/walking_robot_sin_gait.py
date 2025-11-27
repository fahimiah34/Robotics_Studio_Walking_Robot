"""
Isaac Sim 5.1.0 - Quadruped URDF Import with Sinusoidal Gait
- Imports quadruped URDF and saves as USD
- Colors robot orange using displayColor (unbinds existing materials)
- Runs sinusoidal trot gait for 10 seconds
- Then returns robot to standing pose for 2 seconds
- Logs base speed + joint angle/velocity/torque and saves plots as JPG
"""

import omni
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim (GUI mode)
simulation_app = SimulationApp({"headless": False})

import os
import numpy as np
import carb
import matplotlib.pyplot as plt

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom, Gf, UsdShade
import omni.kit.commands
from isaacsim.asset.importer.urdf import _urdf

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
URDF_PATH = r"C:\isaac-assets\walking_robot\walking_robot.urdf"
USD_OUTPUT_PATH = r"C:\isaac-assets\walking_robot\quadruped.usd"
OUTPUT_DIR = os.path.dirname(URDF_PATH)

# Gait parameters
GAIT_FREQUENCY = 0.5  # Hz
HIP_AMPLITUDE = 0.3   # rad
KNEE_AMPLITUDE = 0.5  # rad
PHASE_OFFSET = np.pi / 2  # knee phase vs hip

WALK_DURATION = 10.0   # seconds of walking
STAND_DURATION = 2.0   # seconds of standing after walk

# Leg phase offsets for trot (diagonal pairs in phase)
LEG_PHASES = {
    "fr": 0.0,        # Front-right
    "bl": 0.0,        # Back-left (diagonal pair with fr)
    "fl": np.pi,      # Front-left (opposite)
    "br": np.pi       # Back-right (diagonal pair with fl)
}

# ---------------------------------------------------------------------
# URDF → USD
# ---------------------------------------------------------------------
def import_urdf_to_usd():
    """Import URDF file and convert to USD."""
    print(f"Importing URDF from: {URDF_PATH}")

    stage = omni.usd.get_context().get_stage()

    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False  # quadruped base is free
    import_config.distance_scale = 1.0
    import_config.density = 0.0  # use URDF masses
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 1e4
    import_config.default_position_drive_damping = 1e3

    success, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=URDF_PATH,
        import_config=import_config,
    )

    if not success:
        print("Failed to import URDF")
        return None

    print(f"Successfully imported URDF to: {prim_path}")
    stage.Export(USD_OUTPUT_PATH)
    print(f"Saved USD to: {USD_OUTPUT_PATH}")
    return prim_path

# ---------------------------------------------------------------------
# Orange color via displayColor (unbind materials)
# ---------------------------------------------------------------------
def apply_orange_color(prim_path):
    """
    Force all robot geometry to be Home-Depot-like orange
    using UsdGeom.Gprim displayColor.
    Also unbinds any existing materials so displayColor is visible.
    """
    print("Applying orange displayColor to robot...")
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(prim_path)

    if not robot_prim or not robot_prim.IsValid():
        print(f"[WARN] Prim at {prim_path} not found or invalid")
        return

    # Home Depot-ish orange
    orange = Gf.Vec3f(0.969, 0.565, 0.118)

    def recurse(prim):
        if prim.IsA(UsdGeom.Gprim):  # Mesh, Cube, Cylinder, etc.
            # Unbind any existing material so displayColor takes effect
            binding_api = UsdShade.MaterialBindingAPI(prim)
            binding_api.UnbindAllBindings()

            gprim = UsdGeom.Gprim(prim)
            color_attr = gprim.CreateDisplayColorAttr()
            color_attr.Set([orange])

        for child in prim.GetChildren():
            recurse(child)

    recurse(robot_prim)
    print("Orange color applied.")

# ---------------------------------------------------------------------
# World setup
# ---------------------------------------------------------------------
def setup_simulation_world():
    """Create Isaac Sim World and ground plane."""
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    return world

# ---------------------------------------------------------------------
# Joint angle function (pure kinematics for gait)
# ---------------------------------------------------------------------
def calculate_joint_angle(time, leg_suffix, joint_type):
    """
    Calculate joint angle for sinusoidal gait.
    joint_type: 'hip' or 'knee'
    leg_suffix: 'fr', 'fl', 'br', 'bl'
    """
    leg_phase = LEG_PHASES[leg_suffix]
    omega = 2 * np.pi * GAIT_FREQUENCY

    if joint_type == "hip":
        return HIP_AMPLITUDE * np.sin(omega * time + leg_phase)
    else:  # knee
        return KNEE_AMPLITUDE * np.sin(omega * time + leg_phase + PHASE_OFFSET)

# ---------------------------------------------------------------------
# Main animation loop (walk 10s, then stand 2s)
# ---------------------------------------------------------------------
def animate_quadruped(world, robot):
    print("\nStarting simulation with sinusoidal gait...")
    print("Robot will walk for 10 seconds, then stand for 2 seconds.\n")

    joint_names = robot.dof_names
    print(f"Available joints: {joint_names}")

    joint_index_map = {name: i for i, name in enumerate(joint_names)}

    # Reset + initial pose
    world.reset()
    dt = 1.0 / 60.0  # 60 Hz
    robot.set_world_pose(position=np.array([0.0, 0.0, 0.15]))

    # Wait a couple seconds before walking
    print("Waiting 2 seconds before starting gait...")
    wait_time = 0.0
    while wait_time < 2.0 and simulation_app.is_running():
        world.step(render=True)
        wait_time += dt

    print("Starting gait animation...\n")

    # Logging containers
    t_hist = []
    base_x_hist = []
    speed_hist = []

    joint_pos_hist = []
    joint_vel_hist = []
    joint_torque_hist = []

    last_target_positions = np.zeros(len(joint_names))
    last_base_x = None
    step_count = 0

    time_sim = 0.0

    try:
        # ----------------- WALK PHASE (10s) -----------------
        while simulation_app.is_running() and time_sim < WALK_DURATION:
            target_positions = np.zeros(len(joint_names))

            # Compute joint targets for each leg
            for leg in ["fr", "fl", "br", "bl"]:
                hip_name = f"hip_{leg}"
                knee_name = f"knee_{leg}"

                if hip_name in joint_index_map:
                    hip_angle = calculate_joint_angle(time_sim, leg, "hip")
                    target_positions[joint_index_map[hip_name]] = hip_angle

                if knee_name in joint_index_map:
                    knee_angle = calculate_joint_angle(time_sim, leg, "knee")
                    target_positions[joint_index_map[knee_name]] = knee_angle

            # Apply joint commands
            robot.set_joint_positions(target_positions)

            # Step sim
            world.step(render=True)
            time_sim += dt
            step_count += 1

            # --- Measurements ---
            base_pos, base_orient = robot.get_world_pose()
            base_x = float(base_pos[0])

            if last_base_x is None:
                speed_x = 0.0
            else:
                speed_x = (base_x - last_base_x) / dt
            last_base_x = base_x

            current_joint_pos = robot.get_joint_positions()
            try:
                current_joint_vel = robot.get_joint_velocities()
            except AttributeError:
                current_joint_vel = (target_positions - last_target_positions) / dt

            try:
                current_joint_torque = robot.get_applied_joint_efforts()
            except AttributeError:
                current_joint_torque = np.zeros_like(current_joint_pos)

            t_hist.append(time_sim)
            base_x_hist.append(base_x)
            speed_hist.append(speed_x)

            joint_pos_hist.append(np.array(current_joint_pos))
            joint_vel_hist.append(np.array(current_joint_vel))
            joint_torque_hist.append(np.array(current_joint_torque))

            last_target_positions = target_positions.copy()

            if step_count % 60 == 0:  # ~1 Hz log
                print(
                    f"t = {time_sim:5.2f} s | base x = {base_x: .3f} m | "
                    f"speed ≈ {speed_x: .3f} m/s"
                )

        print("\nFinished walking phase. Returning to standing pose...")

        # ----------------- STAND PHASE (2s) -----------------
        stand_time = 0.0
        standing_positions = np.zeros(len(joint_names))  # all joints to 0 rad

        while simulation_app.is_running() and stand_time < STAND_DURATION:
            robot.set_joint_positions(standing_positions)

            world.step(render=True)
            time_sim += dt
            stand_time += dt

            base_pos, base_orient = robot.get_world_pose()
            base_x = float(base_pos[0])

            if last_base_x is None:
                speed_x = 0.0
            else:
                speed_x = (base_x - last_base_x) / dt
            last_base_x = base_x

            current_joint_pos = robot.get_joint_positions()
            try:
                current_joint_vel = robot.get_joint_velocities()
            except AttributeError:
                current_joint_vel = np.zeros_like(current_joint_pos)
            try:
                current_joint_torque = robot.get_applied_joint_efforts()
            except AttributeError:
                current_joint_torque = np.zeros_like(current_joint_pos)

            t_hist.append(time_sim)
            base_x_hist.append(base_x)
            speed_hist.append(speed_x)

            joint_pos_hist.append(np.array(current_joint_pos))
            joint_vel_hist.append(np.array(current_joint_vel))
            joint_torque_hist.append(np.array(current_joint_torque))

        print("Standing phase complete.\n")

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    # ----------------- STATS -----------------
    if len(t_hist) > 1:
        total_time = t_hist[-1] - t_hist[0]
        total_dist = base_x_hist[-1] - base_x_hist[0]
        avg_speed = total_dist / total_time if total_time > 0 else 0.0
        print(f"\nTotal sim time: {total_time:.2f} s")
        print(f"Forward distance: {total_dist:.3f} m")
        print(f"Average forward speed: {avg_speed:.3f} m/s")

    # ----------------- PLOTTING → SAVE JPG -----------------
    if len(t_hist) == 0:
        print("No data collected, skipping plots.")
        return

    t_hist = np.array(t_hist)

    # 1) Base x and speed
    base_fig = plt.figure(figsize=(8, 6))
    plt.subplot(2, 1, 1)
    plt.plot(t_hist, base_x_hist)
    plt.ylabel("Base x [m]")
    plt.title("Base Position and Forward Speed")

    plt.subplot(2, 1, 2)
    plt.plot(t_hist, speed_hist)
    plt.xlabel("Time [s]")
    plt.ylabel("Forward speed [m/s]")

    base_plot_path = os.path.join(OUTPUT_DIR, "base_speed.jpg")
    base_fig.savefig(base_plot_path, dpi=300, bbox_inches="tight")
    plt.close(base_fig)

    # 2) Joint states
    pos_arr = np.vstack(joint_pos_hist)      # [T, dof]
    vel_arr = np.vstack(joint_vel_hist)      # [T, dof]
    tor_arr = np.vstack(joint_torque_hist)   # [T, dof]

    joint_fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    joint_fig.suptitle("Joint States vs Time")

    for i, name in enumerate(joint_names):
        axs[0].plot(t_hist, pos_arr[:, i], label=name)
        axs[1].plot(t_hist, vel_arr[:, i], label=name)
        axs[2].plot(t_hist, tor_arr[:, i], label=name)

    axs[0].set_ylabel("Angle [rad]")
    axs[1].set_ylabel("Vel [rad/s]")
    axs[2].set_ylabel("Torque [Nm?]")
    axs[2].set_xlabel("Time [s]")

    axs[0].legend(loc="upper right", fontsize=8)

    joint_plot_path = os.path.join(OUTPUT_DIR, "joint_states.jpg")
    joint_fig.savefig(joint_plot_path, dpi=300, bbox_inches="tight")
    plt.close(joint_fig)

    print(f"Saved base/speed plot to: {base_plot_path}")
    print(f"Saved joint states plot to: {joint_plot_path}")

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------
def main():
    print("=" * 60)
    print("Quadruped URDF Import and Sinusoidal Gait Animation")
    print("=" * 60)

    prim_path = import_urdf_to_usd()
    if prim_path is None:
        simulation_app.close()
        return

    print(f"Imported robot prim path: {prim_path}")

    # Set up world + add articulation
    world = setup_simulation_world()
    world.reset()

    robot = world.scene.add(
        Articulation(
            prim_path=prim_path,
            name="quadruped"
        )
    )
    robot.initialize()

    # Apply orange color AFTER robot exists
    apply_orange_color(prim_path)

    print("\nRobot initialized:")
    print(f"  - DOF count: {robot.num_dof}")
    print(f"  - Joint names: {robot.dof_names}")

    # Run gait + logging + plots
    animate_quadruped(world, robot)

    simulation_app.close()


if __name__ == "__main__":
    main()
