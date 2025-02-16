package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.utils.Constants.OperatorConstants;;

public class TeleOp {
    final static Joystick driverJoystick = new Joystick(OperatorConstants.DRIVER_PORT);
    final static Joystick codriverJoystick = new Joystick(OperatorConstants.CODRIVER_PORT);

    final static JoystickButton zeroHdgBtn = new JoystickButton(driverJoystick, OperatorConstants.ZERO_HDG);
    final static JoystickButton calibrateBtn = new JoystickButton(codriverJoystick, OperatorConstants.CALIBRATE);
    final static JoystickButton calibrateBtn1 = new JoystickButton(codriverJoystick, OperatorConstants.CALIBRATE1);
    final static JoystickButton auto = new JoystickButton(codriverJoystick, OperatorConstants.AUTO);
    final static JoystickButton rollers = new JoystickButton(codriverJoystick, OperatorConstants.ROLLERS);

    public static void initialize() {
        RobotContainer.swerve.setDefaultCommand(new SwerveDriveJoystick(
            RobotContainer.swerve,
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_X),
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_Y),
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_Z),
            () -> !driverJoystick.getRawButton(OperatorConstants.ROBOT_ORIENTED)));
        zeroHdgBtn.onTrue(new InstantCommand(() -> RobotContainer.swerve.zeroHeading()));
        
        calibrateBtn.and(calibrateBtn1).onTrue(RobotContainer.calibrateSubystems());

        if (driverJoystick.getRawAxis(OperatorConstants.DRIVER_X) + driverJoystick.getRawAxis(OperatorConstants.DRIVER_Y) + driverJoystick.getRawAxis(OperatorConstants.DRIVER_Z) == 0){
            auto.toggleOnTrue(RobotContainer.autonomous);
        }

        RobotContainer.claw.setAngle(codriverJoystick.getRawAxis(OperatorConstants.CODRIVER_A));
        RobotContainer.claw.setClaw(codriverJoystick.getRawAxis(OperatorConstants.CODRIVER_X));
        RobotContainer.arm.setArm(codriverJoystick.getRawAxis(OperatorConstants.CODRIVER_Z));
        RobotContainer.elevator.setSpeed(codriverJoystick.getRawAxis(OperatorConstants.CODRIVER_Y));

        rollers.toggleOnTrue(RobotContainer.getRollersCommand());
    }


}

