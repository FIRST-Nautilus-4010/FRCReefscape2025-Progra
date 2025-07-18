package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.SwerveDriveJoystick;

public class TeleOp {
    //--------Constants--------

    // Joystick settings
    public static final int DRIVER_PORT = 0;

      // Sticks
    public static final int DRIVER_X = 1;
    public static final int DRIVER_Y = 0;
    public static final int DRIVER_Z = 4;

      // Buttons
    public static final int ZERO_HDG = 1;
    public static final int ROBOT_ORIENTED = 5;

    //--------Co-Driver constants--------

      // Joystick settings
    public static final int CODRIVER_PORT = 1;

      // Sticks
    public static final int CODRIVER_X = 1;
    public static final int CODRIVER_Y = 0;
    public static final int CODRIVER_Z = 4;
    public static final int CODRIVER_A = 3;
    
      // Buttons
    public static final int CALIBRATE = 6;
    public static final int CALIBRATE1 = 7;
    public static final int AUTO = 8;
    public static final int ROLLERS = 5;

    final static Joystick driverJoystick = new Joystick(DRIVER_PORT); // <-- Driver joystick initialization.
    final static Joystick codriverJoystick = new Joystick(CODRIVER_PORT); // <-- Co-driver joystick initialization.

    final static JoystickButton zeroHdgBtn = new JoystickButton(driverJoystick, ZERO_HDG); // <-- Button to zero the robot's heading.
    final static JoystickButton calibrateBtn = new JoystickButton(codriverJoystick, CALIBRATE); // <-- Button for calibration.
    final static JoystickButton calibrateBtn1 = new JoystickButton(codriverJoystick, CALIBRATE1); // <-- Additional calibration button.
    final static JoystickButton auto = new JoystickButton(codriverJoystick, AUTO); // <-- Button to trigger autonomous mode.
    final static JoystickButton rollers = new JoystickButton(codriverJoystick, ROLLERS); // <-- Button to control rollers.

    public static void initialize() {
        // Sets the default command for the swerve subsystem to joystick control.
        RobotContainer.swerve.setDefaultCommand(new SwerveDriveJoystick(
            RobotContainer.swerve, // <-- Swerve subsystem instance.
            () -> -driverJoystick.getRawAxis(DRIVER_X), // <-- X-axis input from the driver joystick.
            () -> -driverJoystick.getRawAxis(DRIVER_Y), // <-- Y-axis input from the driver joystick.
            () -> -driverJoystick.getRawAxis(DRIVER_Z), // <-- Z-axis (rotation) input from the driver joystick.
            () -> !driverJoystick.getRawButton(ROBOT_ORIENTED))); // <-- Determines if the robot is in robot-oriented mode.

        // Maps the zero heading button to an InstantCommand to reset the robot's heading.
        zeroHdgBtn.onTrue(new InstantCommand(() -> RobotContainer.swerve.zeroHeading())); // <-- Resets the robot's heading.
    }
}