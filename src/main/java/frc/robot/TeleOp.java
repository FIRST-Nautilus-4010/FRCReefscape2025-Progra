package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.utils.Constants.OperatorConstants;

public class TeleOp {
    final static Joystick driverJoystick = new Joystick(OperatorConstants.DRIVER_PORT); // <-- Driver joystick initialization.
    final static Joystick codriverJoystick = new Joystick(OperatorConstants.CODRIVER_PORT); // <-- Co-driver joystick initialization.

    final static JoystickButton zeroHdgBtn = new JoystickButton(driverJoystick, OperatorConstants.ZERO_HDG); // <-- Button to zero the robot's heading.
    final static JoystickButton calibrateBtn = new JoystickButton(codriverJoystick, OperatorConstants.CALIBRATE); // <-- Button for calibration.
    final static JoystickButton calibrateBtn1 = new JoystickButton(codriverJoystick, OperatorConstants.CALIBRATE1); // <-- Additional calibration button.
    final static JoystickButton auto = new JoystickButton(codriverJoystick, OperatorConstants.AUTO); // <-- Button to trigger autonomous mode.
    final static JoystickButton rollers = new JoystickButton(codriverJoystick, OperatorConstants.ROLLERS); // <-- Button to control rollers.

    public static void initialize() {
        // Sets the default command for the swerve subsystem to joystick control.
        RobotContainer.swerve.setDefaultCommand(new SwerveDriveJoystick(
            RobotContainer.swerve, // <-- Swerve subsystem instance.
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_X), // <-- X-axis input from the driver joystick.
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_Y), // <-- Y-axis input from the driver joystick.
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_Z), // <-- Z-axis (rotation) input from the driver joystick.
            () -> !driverJoystick.getRawButton(OperatorConstants.ROBOT_ORIENTED))); // <-- Determines if the robot is in robot-oriented mode.

        // Maps the zero heading button to an InstantCommand to reset the robot's heading.
        zeroHdgBtn.onTrue(new InstantCommand(() -> RobotContainer.swerve.zeroHeading())); // <-- Resets the robot's heading.
    }
}