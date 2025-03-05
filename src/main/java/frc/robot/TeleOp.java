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

    static double lastAnglePos = RobotContainer.claw.getAngle();
    static double lastArmPos = RobotContainer.arm.getAngle();

    public static void initialize() {
        RobotContainer.swerve.setDefaultCommand(new SwerveDriveJoystick(
            RobotContainer.swerve,
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_X),
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_Y),
            () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_Z),
            () -> !driverJoystick.getRawButton(OperatorConstants.ROBOT_ORIENTED)));
        zeroHdgBtn.onTrue(new InstantCommand(() -> RobotContainer.swerve.zeroHeading()));
        
        //calibrateBtn.and(calibrateBtn1).onTrue(RobotContainer.calibrateSubystems());

        //auto.toggleOnTrue(RobotContainer.autonomous);
        
        RobotContainer.claw.setDefaultCommand( 
            new InstantCommand(() -> moveClaw(), RobotContainer.claw)
        );
        RobotContainer.arm.setDefaultCommand(
            new InstantCommand(() -> moveArm(), RobotContainer.arm)
        );
        //RobotContainer.elevator.setDefaultCommand(
        //    new InstantCommand(() -> moveElevator(), RobotContainer.elevator)
        //);
       

       rollers.toggleOnTrue(new InstantCommand(() -> RobotContainer.claw.setRollersSpeed(1)));
       rollers.toggleOnFalse(new InstantCommand(() -> RobotContainer.claw.setRollersSpeed(0)));
        
    }

    private static void moveClaw() {
        
        if (Math.abs(codriverJoystick.getRawAxis(0)) > .1) {
            RobotContainer.claw.setAngle(codriverJoystick.getRawAxis(0)*.6);
            lastAnglePos = RobotContainer.claw.getAngle();
        } else {
            RobotContainer.claw.setAnglePos(lastAnglePos);
        }
        if (Math.abs(codriverJoystick.getRawAxis(5)) > .1){
            RobotContainer.claw.setClaw((codriverJoystick.getRawAxis(5) * .1));
        } else {
            RobotContainer.claw.setClaw(0);
        }
    }

    private static void moveArm() {
        if (codriverJoystick.getRawAxis(3) > 0) {
            RobotContainer.arm.set(codriverJoystick.getRawAxis(3)*.2);
            lastArmPos = RobotContainer.arm.getAngle();
        } else if (codriverJoystick.getRawAxis(2) > 0) {
            RobotContainer.arm.set(-codriverJoystick.getRawAxis(2)*.2);
            lastArmPos = RobotContainer.arm.getAngle();
        } else {
            RobotContainer.arm.setPos(lastAnglePos);
        }
    }

    private static void moveElevator() {
        if (driverJoystick.getRawAxis(3) > 0) {
            RobotContainer.elevator.set(driverJoystick.getRawAxis(3));
        } else if (driverJoystick.getRawAxis(2) > 0) {
            RobotContainer.elevator.set(-driverJoystick.getRawAxis(2));
        } else {
            RobotContainer.elevator.set(.13);
        }
    }


}

