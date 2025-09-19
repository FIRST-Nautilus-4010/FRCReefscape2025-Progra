package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevators.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;

public final class SubsystemManager {

    private static Swerve swerve = null;
    private static final Shoulder shoulder = new Shoulder();
    private static final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private static final EndEffector endEffector = new EndEffector();

    public static RobotState robotState = RobotState.TRAVEL;
    private static RobotState lastState = null;

    public static void initialize(Swerve swerveSubsystem) {
        swerve = swerveSubsystem;
        
        SwerveModule.setMassCenterHeightSupplier(() -> (shoulder.getHeight() + elevator.getHeight() + 1));
        ShoulderController.setRobotAccSupplier(() -> swerve.getAccelX());
        
        scheduleState(RobotState.TRAVEL);
    }

    private static Pose2d getClosestSource() {
        return swerve.getPose();
    }
    
    private static Pose2d getClosestL() {
        return swerve.getPose();
    }

    private static void travel() {
        new ParallelCommandGroup(
            new InstantCommand(() -> setState(RobotState.TRAVEL)),
            //shoulder.rotateToTravel(() -> elevator.getHeight()),
            elevator.moveToTravel(() -> shoulder.getAngle())
        ).schedule();
    }

    private static void intake() {
        new ParallelCommandGroup(
            new InstantCommand(() -> setState(RobotState.INTAKE)),
            //shoulder.rotateToIntake(() -> elevator.getHeight()),
            elevator.moveToIntake(() -> shoulder.getAngle())
            //endEffector.intake()
            //swerve.driveTo(getClosestSource())
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }

    private static void putL1() {
        new SequentialCommandGroup(
            new InstantCommand(() -> setState(RobotState.PUT_L1)),
            new ParallelCommandGroup(
                //shoulder.rotateToL1(() -> elevator.getHeight()),
                elevator.moveToL1(() -> shoulder.getAngle())
                //swerve.driveTo(getClosestL())
            )
            //endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }

    private static void putL2() {
        new SequentialCommandGroup(
            new InstantCommand(() -> setState(RobotState.PUT_L2)),
            new ParallelCommandGroup(
                //shoulder.rotateToL2L3(() -> elevator.getHeight()),
                elevator.moveToL2(() -> shoulder.getAngle())
                //swerve.driveTo(getClosestL())
            )
            //elevator.putL2L3(() -> shoulder.getAngle()),
            //endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }
    
    private static void putL3() {
        new SequentialCommandGroup(
            new InstantCommand(() -> setState(RobotState.PUT_L3)),
            new ParallelCommandGroup(
                //shoulder.rotateToL2L3(() -> elevator.getHeight()),
                elevator.moveToL3(() -> shoulder.getAngle())
                //swerve.driveTo(getClosestL())
            )
            //elevator.putL2L3(() -> shoulder.getAngle()),
            //endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }

    private static void putL4() {
        new SequentialCommandGroup(
            new InstantCommand(() -> setState(RobotState.PUT_L4)),
            new ParallelCommandGroup(
                //shoulder.rotateToL4(() -> elevator.getHeight()),
                elevator.moveToL4(() -> shoulder.getAngle())
                //swerve.driveTo(getClosestL())
            )
            //shoulder.putL4(() -> elevator.getHeight()),
            //endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }

    private static void setState(RobotState state) {
        robotState = state;
    }

    public static void executeState(RobotState state) {
        CommandScheduler.getInstance().cancelAll();
        scheduleState(state);
    }

    public static void scheduleState(RobotState state) {
        switch (state) {
            case TRAVEL:
                travel();
                break;
            case INTAKE:
                intake();
                break;
            case PUT_L1:
                putL1();
                break;
            case PUT_L2:
                putL2();
                break;
            case PUT_L3:
                putL3();
                break;
            case PUT_L4:
                putL4();
                break;
        }
    }

    public static void update() {
        SmartDashboard.putString("Robot State", robotState.toString());
        SmartDashboard.putString("Last Robot State", lastState != null ? lastState.toString() : "None");
    }
}