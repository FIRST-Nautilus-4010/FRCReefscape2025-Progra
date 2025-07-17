package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants.AutonomousConstants;
import frc.robot.utils.Constants.ChassisConstants;

public class RobotContainer {
  public static final Vision  vision = new Vision();
  public static final Swerve swerve = new Swerve(true);

  public static Command getAutonomouCmd() {
    return goTo(swerve.getPose(), null, new Pose2d(4.6, 1.5, swerve.getRotation2d()));
  }


  public static Command goTo(Pose2d start, List<Translation2d> intermediatePoints, Pose2d end) {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutonomousConstants.MAX_SPD, AutonomousConstants.MAX_ACCEL); // <-- Configures the trajectory with maximum speed and acceleration.
    swerve.resetOdometry(new Pose2d()); // <-- Resets the robot's odometry to the initial pose.

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        start, // <-- Starting pose of the trajectory.
        intermediatePoints, // <-- List of intermediate waypoints for the trajectory.
        end, // <-- Ending pose of the trajectory.
        trajectoryConfig // <-- Trajectory configuration settings.
    );
    publishTrajectory(trajectory); // <-- Publishes the generated trajectory for debugging or visualization.

    PIDController xController = new PIDController(AutonomousConstants.P, AutonomousConstants.I, AutonomousConstants.D); // <-- PID controller for X-axis control.
    PIDController yController = new PIDController(AutonomousConstants.P, AutonomousConstants.I, AutonomousConstants.D); // <-- PID controller for Y-axis control.
    ProfiledPIDController zController = new ProfiledPIDController(AutonomousConstants.P_Z, AutonomousConstants.I_Z, AutonomousConstants.D_Z, AutonomousConstants.Z_CONTROLER); // <-- Profiled PID controller for Z-axis (rotation) control.
    zController.enableContinuousInput(-Math.PI, Math.PI); // <-- Enables continuous input for rotation angles between -π and π.

    Command command = new SwerveControllerCommand(
        trajectory, // <-- The trajectory to follow.
        swerve::getPose, // <-- Function to get the robot's current pose.
        ChassisConstants.KINEMATICS, // <-- Kinematics configuration for the swerve drive.
        xController, // <-- PID controller for X-axis control.
        yController, // <-- PID controller for Y-axis control.
        zController, // <-- Profiled PID controller for Z-axis control.
        swerve::setStates, // <-- Function to set the swerve module states.
        swerve // <-- The swerve subsystem.
    );

    return command; // <-- Returns the command to execute the trajectory.
  }

  public static Command getTestCommand() {
    return new InstantCommand(() -> goTo(swerve.getPose(), null, new Pose2d(1, 1.5, new Rotation2d(52)))); // <-- Creates a test command to move the robot to a specific pose.
  }

  private static void publishTrajectory(Trajectory trajectory) {
    StructArrayPublisher<Pose2d> trajectoryPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Trajectory", Pose2d.struct).publish();

    List<Pose2d> points = new ArrayList<>();
    for (var state : trajectory.getStates()) {
      points.add(state.poseMeters);
    }
    trajectoryPublisher.set(points.toArray(new Pose2d[0]));
  }
}
