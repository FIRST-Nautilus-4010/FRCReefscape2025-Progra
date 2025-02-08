// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;

public class RobotContainer {
  public static final Swerve swerve = new Swerve(true);

  public static Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static Command getTestCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.AutonomousConstants.MAX_SPD, Constants.AutonomousConstants.MAX_ACCEL);
    swerve.resetOdometry(new Pose2d());

    Trajectory trajectory = generateTrajectory(trajectoryConfig);
    publishTrajectory(trajectory);

    PIDController xController = new PIDController(Constants.AutonomousConstants.P, Constants.AutonomousConstants.I, Constants.AutonomousConstants.D);
    PIDController yController = new PIDController(Constants.AutonomousConstants.P, Constants.AutonomousConstants.I, Constants.AutonomousConstants.D);
    ProfiledPIDController zController = new ProfiledPIDController(Constants.AutonomousConstants.P_Z, Constants.AutonomousConstants.I_Z, Constants.AutonomousConstants.D_Z, Constants.AutonomousConstants.Z_CONTROLER);
    zController.enableContinuousInput(-Math.PI, Math.PI);

    return new SwerveControllerCommand(
        trajectory,
        swerve::getPose,
        Constants.ChassisConstants.KINEMATICS,
        xController,
        yController,
        zController,
        swerve::setStates,
        swerve
    );
  }

  private static Trajectory generateTrajectory(TrajectoryConfig config) {
    return TrajectoryGenerator.generateTrajectory(
        swerve.getPose(),
        List.of(
            new Translation2d(3, 0),
            new Translation2d(3, -1.5),
            new Translation2d(1.5, -1.5),
            new Translation2d(1.5, 0),
            new Translation2d(4.5, 0),
            new Translation2d(4.5, 1.5),
            new Translation2d(3, 1.5),
            new Translation2d(3, 0),
            new Translation2d(6, -1.5),
            new Translation2d(7.5, -1.5),
            new Translation2d(7.5, 0)
        ),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        config
    );
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
