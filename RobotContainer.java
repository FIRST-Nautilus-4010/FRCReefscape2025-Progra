package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
  private final Swerve swerve;
  private final ElevatorSubsystem elevator;

  private final XboxController driverJoystick;
  private final XboxController codriverJoystick;

  public RobotContainer() {
    swerve = new Swerve(true);
    elevator = new ElevatorSubsystem();

    driverJoystick = new XboxController(0);
    codriverJoystick = new XboxController(1);

    configureBindings();
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

    Trigger aButton = new JoystickButton(codriverJoystick, XboxController.Button.kA.value);
    aButton.onTrue(elevator.elevate());

    Trigger bButton = new JoystickButton(codriverJoystick, XboxController.Button.kB.value);
    bButton.onTrue(elevator.descend());

    Trigger xButton = new JoystickButton(codriverJoystick, XboxController.Button.kX.value);
    xButton.onTrue(elevator.stop());

    Trigger yButton = new JoystickButton(codriverJoystick, XboxController.Button.kY.value);
    yButton.onTrue(elevator.goToL1());

    Trigger startButton = new JoystickButton(codriverJoystick, XboxController.Button.kStart.value);
    startButton.onTrue(elevator.goToL2());

  } 

  public Command getTestCommand() {
    return new InstantCommand();
  }
}

