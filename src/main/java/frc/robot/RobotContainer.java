package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.commands.SwerveDriveJoystick;

public class RobotContainer {
  private final Swerve swerve;

  private final XboxController driverJoystick;
  private final XboxController codriverJoystick;

  public RobotContainer() {
    swerve = new Swerve(true);

    driverJoystick = new XboxController(0);
    codriverJoystick = new XboxController(1);
    configureBindings();

    SubsystemManager.initialize(swerve);
  }

  public Command getAutonomouCmd() {
    return new InstantCommand();
  }


  public void configureBindings() {
    // Sets the default command for the swerve subsystem to joystick control.
    swerve.setDefaultCommand(new SwerveDriveJoystick(
        swerve, // <-- Swerve subsystem instance.
        () -> driverJoystick.getLeftX(), // <-- X-axis input from the driver joystick.
        () -> driverJoystick.getLeftY(), // <-- Y-axis input from the driver joystick.
        () -> driverJoystick.getRightX(), // <-- Z-axis (rotation) input from the driver joystick.
        () -> !driverJoystick.getXButton(), // <-- Determines if the robot is in robot-oriented mode.
        () -> driverJoystick.getAButton()
    ));

    Trigger intake = new Trigger(() -> codriverJoystick.getLeftBumperButtonPressed());
    intake.onTrue(new InstantCommand(() -> SubsystemManager.scheduleState(RobotState.INTAKE)));

    Trigger putL1 = new Trigger(() -> codriverJoystick.getPOV() == 0);
    putL1.onTrue(new InstantCommand(() -> SubsystemManager.scheduleState(RobotState.PUT_L1)));

    Trigger putL2 = new Trigger(() -> codriverJoystick.getPOV() == 90);
    putL2.onTrue(new InstantCommand(() -> SubsystemManager.scheduleState(RobotState.PUT_L2)));

    Trigger putL3 = new Trigger(() -> codriverJoystick.getPOV() == 180);
    putL3.onTrue(new InstantCommand(() -> SubsystemManager.scheduleState(RobotState.PUT_L3)));

    Trigger putL4 = new Trigger(() -> codriverJoystick.getPOV() == 270);
    putL4.onTrue(new InstantCommand(() -> SubsystemManager.scheduleState(RobotState.PUT_L4)));

    Trigger travel = new Trigger(() -> codriverJoystick.getXButton());
    travel.onTrue(new InstantCommand(() -> SubsystemManager.executeState(RobotState.TRAVEL)));
  } 

  public Command getTestCommand() {
    return new InstantCommand();
  }
}