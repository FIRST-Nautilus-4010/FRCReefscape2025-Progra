// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.commands;



import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.swerve.PoseTracker;
import frc.robot.subsystems.swerve.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTo extends Command { 
  private final Pose2d target;

  private final Swerve swerve;
  private final PoseTracker poseTracker;

  private final HolonomicDriveController controller;

  public DriveTo(Pose2d target, Swerve swerve, PoseTracker poseTracker) {
    this.target = target;
    this.swerve = swerve;
    this.poseTracker = poseTracker;

    ProfiledPIDController thetaController = new ProfiledPIDController(AutonomousConstants.P_Z, AutonomousConstants.I_Z, AutonomousConstants.D_Z, AutonomousConstants.Z_CONTROLER);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(
            new PIDController(AutonomousConstants.P_X, AutonomousConstants.I_X, AutonomousConstants.D_X),
            new PIDController(AutonomousConstants.P_Y, AutonomousConstants.I_Y, AutonomousConstants.D_Y),
            thetaController
        );

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    var chassisSpeeds = controller.calculate(poseTracker.getPose(), target, AutonomousConstants.MAX_SPD, target.getRotation());
    swerve.drive(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    if (poseTracker.getPose().getTranslation().getDistance(target.getTranslation()) < AutonomousConstants.POS_TOLERANCE &&
        Math.abs(poseTracker.getPose().getRotation().getRadians() - target.getRotation().getRadians()) < AutonomousConstants.ANG_TOLERANCE) {
      return true;
    }
    return false;
  }
}
