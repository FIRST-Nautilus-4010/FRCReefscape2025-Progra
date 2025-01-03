package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.utils.Constants;

public class TeleOp {
    final Joystick driverJoystick = new Joystick(Constants.OperatorConstants.DRIVER_PORT);
    final Joystick codriverJoystick = new Joystick(Constants.OperatorConstants.CODRIVER_PORT);

    final JoystickButton zeroHdgBtn = new JoystickButton(driverJoystick, Constants.OperatorConstants.ZERO_HDG);

    public TeleOp() {
        RobotContainer.swerve.setDefaultCommand(new SwerveDriveJoystick(
            RobotContainer.swerve,
            () -> -driverJoystick.getRawAxis(Constants.OperatorConstants.DRIVER_X),
            () -> -driverJoystick.getRawAxis(Constants.OperatorConstants.DRIVER_Y),
            () -> -driverJoystick.getRawAxis(Constants.OperatorConstants.DRIVER_Z),
            () -> driverJoystick.getRawButton(Constants.OperatorConstants.ROBOT_ORIENTED)));
        zeroHdgBtn.onTrue(new InstantCommand(() -> RobotContainer.swerve.zeroHeading()));
    }

    
}
