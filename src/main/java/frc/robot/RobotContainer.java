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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.Autonomous;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.AutonomousConstants;
import frc.robot.utils.Constants.ChassisConstants;
import frc.robot.utils.Constants.ClawConstants;

public class RobotContainer {
  public static final Swerve swerve = new Swerve(true);
  public static final Elevator elevator = new Elevator(false);
  public static final Claw claw = new Claw(false, false, false);
  public static final Arm arm = new Arm(false);
  public static Command calibrateSubystems() {
    return new InstantCommand(
        () -> {
          elevator.calibrate();
          claw.calibrate();
          arm.calibrate();
        }
    );
  }

  public static Command getAutonomousCommand() {
    return new Autonomous();
  }

  public static void goTo(Pose2d start, List<Translation2d> intermediatePoints, Pose2d end, Runnable... functions) {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutonomousConstants.MAX_SPD, AutonomousConstants.MAX_ACCEL);
    swerve.resetOdometry(new Pose2d());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      start,
      intermediatePoints,
      end,
      trajectoryConfig
    );;
    publishTrajectory(trajectory);

    PIDController xController = new PIDController(AutonomousConstants.P, AutonomousConstants.I, AutonomousConstants.D);
    PIDController yController = new PIDController(AutonomousConstants.P, AutonomousConstants.I, AutonomousConstants.D);
    ProfiledPIDController zController = new ProfiledPIDController(AutonomousConstants.P_Z, AutonomousConstants.I_Z, AutonomousConstants.D_Z, AutonomousConstants.Z_CONTROLER);
    zController.enableContinuousInput(-Math.PI, Math.PI);

    Command command = new SwerveControllerCommand(
        trajectory,
        swerve::getPose,
        ChassisConstants.KINEMATICS,
        xController,
        yController,
        zController,
        swerve::setStates,
        swerve
    );

    command.initialize();
    while (!command.isFinished()) {
      command.execute();
      for (Runnable function : functions){
        function.run();
      }
    }
    command.end(false);
  }

  public static boolean hasCoral() {
    return claw.getClawAmp() > ClawConstants.CLAW_AMP_THRESHOLD;
  }

  public static void goToSource(int sourceId) {
    try {
        goTo(swerve.getPose(), null, AutonomousConstants.SOURCE_POS[sourceId].toPose2d(),
            () -> arm.setArmPosition(ArmConstants.SOURCE_ANGLE),
            () -> claw.setAnglePosition(ClawConstants.SOURCE_ANGLE),
            () -> elevator.setPosition(AutonomousConstants.SOURCE_POS[sourceId].getZ())
        );

        claw.setClawPosition(ClawConstants.SOURCE_POSITION);
        claw.setRollersSpeed(1);
    } catch (Exception e) {
        System.err.println("Error al ir al source: " + e.getMessage());
    }
  }

  public static void goToReef(int reefId) {
    try {
        goTo(swerve.getPose(), null, AutonomousConstants.REEF_POS[reefId].toPose2d(),
            () -> arm.setArmPosition(ArmConstants.REEF_ANGLE),
            () -> claw.setAnglePosition(ClawConstants.REEF_ANGLE),
            () -> elevator.setPosition(AutonomousConstants.REEF_POS[reefId].getZ())
        );

        claw.setClawPosition(ClawConstants.REEF_POSITION);
        claw.setRollersSpeed(-1);
    } catch (Exception e) {
        System.err.println("Error al ir al arrecife: " + e.getMessage());
    }
  }

  public static Command getTestCommand() {
    return new InstantCommand(() -> goTo(swerve.getPose(), null, new Pose2d(1, 1.5, new Rotation2d(52))));
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
