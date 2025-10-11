package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevators.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderController;
import frc.robot.subsystems.swerve.PoseTracker;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.commands.SwerveDriveJoystick;

public final class SubsystemManager {

    private final Shoulder shoulder = new Shoulder();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final EndEffector endEffector = new EndEffector();
    private final PoseTracker posTracker;

    private RobotState robotState = RobotState.TRAVEL;

    public SubsystemManager(Swerve swerve) {
        posTracker = new PoseTracker(swerve);

        SwerveModule.setMassCenterHeightSupplier(() -> (shoulder.getHeight() + elevator.getHeight() + 1));
        ShoulderController.setRobotAccSupplier(() -> swerve.getAccelX());
        
        scheduleState(RobotState.TRAVEL);
    }

    private Pose2d getClosestSource() {
        return posTracker.getPose();
    }
    
    private Pose3d getClosestL() {
        return new Pose3d(posTracker.getPose().getX(), posTracker.getPose().getY(), 4.0, new Rotation3d());
    }

    private void travel() {
        new ParallelCommandGroup(
            new InstantCommand(() -> setState(RobotState.TRAVEL)),
            shoulder.rotateToTravel(() -> elevator.getHeight()),
            elevator.moveToTravel(() -> shoulder.getAngle())
        ).schedule();
    }

    private void intakeCoral() {
        new ParallelCommandGroup(
            new InstantCommand(() -> setState(RobotState.INTAKE_CORAL)),
            //shoulder.rotateToIntake(() -> elevator.getHeight()),
            elevator.moveToIntake(() -> shoulder.getAngle())
            //endEffector.intake()
            //swerve.driveTo(getClosestSource())
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }

    private void placeL1() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                shoulder.rotateToL1(() -> elevator.getHeight()),
                elevator.moveToL1(() -> shoulder.getAngle())
                //swerve.driveTo(getClosestL())
            )
            //endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }

    private void placeL2() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                shoulder.rotateToL2L3(() -> elevator.getHeight()),
                elevator.moveToL2(() -> shoulder.getAngle())
                //swerve.driveTo(getClosestL())
            ),
            elevator.placeL2(() -> shoulder.getAngle())
            //endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }
    
    private void placeL3() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                shoulder.rotateToL2L3(() -> elevator.getHeight()),
                elevator.moveToL3(() -> shoulder.getAngle())
                //swerve.driveTo(getClosestL())
            ),
            elevator.placeL3(() -> shoulder.getAngle())
            //endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }

    private void placeL4() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                shoulder.rotateToL4(() -> elevator.getHeight()),
                elevator.moveToL4(() -> shoulder.getAngle())
                //swerve.driveTo(getClosestL())
            ),
            shoulder.placeL4(() -> elevator.getHeight())
            //endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }


    private void setState(RobotState state) {
        robotState = state;
    }

    public void executeState(RobotState state) {
        CommandScheduler.getInstance().cancelAll();
        scheduleState(state);
    }

    public void scheduleState(RobotState state) {
        switch (state) {
            case TRAVEL:
                travel();
                break;
            case INTAKE_CORAL:
                intakeCoral();
                break;
            case PLACE_L1:
                placeL1();
                break;
            case PLACE_L2:
                placeL2();
                break;
            case PLACE_L3:
                placeL3();
                break;
            case PLACE_L4:
                placeL4();
                break;
            default:
                travel();
                break;
        }
    }

    public void periodic() {
        posTracker.periodic();

        SmartDashboard.putString("Robot State", robotState.toString());
    }
}