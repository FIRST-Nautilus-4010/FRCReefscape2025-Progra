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

    static double lastArmPos = RobotContainer.arm.getAngle();

    public static void initialize() {
        RobotContainer.swerve.setDefaultCommand(new SwerveDriveJoystick(
            RobotContainer.swerve,
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_X),
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_Y),
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_Z),
            () -> !driverJoystick.getRawButton(OperatorConstants.ROBOT_ORIENTED)));
        zeroHdgBtn.onTrue(new InstantCommand(() -> RobotContainer.swerve.zeroHeading()));

        RobotContainer.arm.setDefaultCommand(
            new InstantCommand(() -> moveArm(), RobotContainer.arm)
        );

        RobotContainer.elevator.setDefaultCommand(
            new InstantCommand(() -> moveElevator(), RobotContainer.elevator)
        );
       

        
    }

    private static void moveArm() {
        if (Math.abs(codriverJoystick.getRawAxis(3)) > 0) {
            RobotContainer.arm.set(codriverJoystick.getRawAxis(3)*.2);
            lastArmPos = RobotContainer.arm.getAngle();
            RobotContainer.arm.runToPosition(false);
        } else if (Math.abs(codriverJoystick.getRawAxis(2)) > 0) {
            RobotContainer.arm.set(-codriverJoystick.getRawAxis(2)*.2);
            lastArmPos = RobotContainer.arm.getAngle();
            RobotContainer.arm.runToPosition(false);
        } else {
            RobotContainer.arm.stop();
            RobotContainer.arm.runToPosition(false);
        }
    }

    private static void moveElevator() {
        if (Math.abs(driverJoystick.getRawAxis(3)) > 0) {
            RobotContainer.elevator.set(driverJoystick.getRawAxis(3));
            RobotContainer.elevator.runToPosition(false);
        } else if (Math.abs(driverJoystick.getRawAxis(2)) > 0) {
            RobotContainer.elevator.set(-driverJoystick.getRawAxis(2));
            RobotContainer.elevator.runToPosition(false);
        } else {
            RobotContainer.elevator.stop();
            RobotContainer.elevator.runToPosition(false);
        }
    }


}