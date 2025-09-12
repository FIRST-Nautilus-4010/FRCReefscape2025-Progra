package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.outake.OutakeSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
  private final Swerve swerve;
  private final ElevatorSubsystem elevator;
  private final ShoulderSubsystem shoulder;
  private final OutakeSubsystem outake;

  private final XboxController driverJoystick;
  private final XboxController codriverJoystick;

  public RobotContainer() {
    swerve = new Swerve(true);
    elevator = new ElevatorSubsystem();
    shoulder = new ShoulderSubsystem(elevator.getIO());
    outake = new OutakeSubsystem();

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
    aButton.onTrue(shoulder.spinRight());

    Trigger bButton = new JoystickButton(codriverJoystick, XboxController.Button.kB.value);
    bButton.onTrue(shoulder.spinLeft());

    Trigger yButton = new JoystickButton(codriverJoystick, XboxController.Button.kY.value);
    yButton.onTrue(elevator.elevate());

    Trigger startButton = new JoystickButton(codriverJoystick, XboxController.Button.kStart.value);
    startButton.onTrue(elevator.descend());

    Trigger xButton = new JoystickButton(codriverJoystick, XboxController.Button.kX.value);
    xButton.onTrue(new ParallelCommandGroup(
      shoulder.stop(),
      elevator.stop(),
      outake.stop()
    ));

    Trigger leftBumperButton = new JoystickButton(codriverJoystick, XboxController.Button.kLeftBumper.value);
    leftBumperButton.onTrue(new ParallelCommandGroup(
      shoulder.goToIntake(),
      elevator.goToIntake(),
      outake.intake()
    ));

    Trigger rightBumperButton = new JoystickButton(codriverJoystick, XboxController.Button.kRightBumper.value);
    rightBumperButton.onTrue(outake.outake());

    Trigger dpadUp = new Trigger(() -> codriverJoystick.getPOV() == 0);
    dpadUp.onTrue(new ParallelCommandGroup(
      shoulder.goToL4(),
      elevator.goToL4()
    ));

    Trigger dpadRight = new Trigger(() -> codriverJoystick.getPOV() == 90);
    dpadRight.onTrue(new ParallelCommandGroup(
      shoulder.goToL3(),
      elevator.goToL3()
    ));

    Trigger dpadDown = new Trigger(() -> codriverJoystick.getPOV() == 180);
    dpadDown.onTrue(new ParallelCommandGroup(
      shoulder.goToL2(),
      elevator.goToL2()
    ));

    Trigger dpadLeft = new Trigger(() -> codriverJoystick.getPOV() == 270);
    dpadLeft.onTrue(new ParallelCommandGroup(
      shoulder.goToL1(),
      elevator.goToL1()
    ));

    Trigger rightJoystickDown = new Trigger(() -> codriverJoystick.getRightY() < -0.5);
    rightJoystickDown.onTrue(shoulder.put());

    Trigger leftJoystickDown = new Trigger(() -> codriverJoystick.getLeftY() < -0.5);
    leftJoystickDown.onTrue(new SequentialCommandGroup(
      shoulder.put(),
      outake.outake()
    ));
  } 

  public Command getTestCommand() {
    return new InstantCommand();
  }
}