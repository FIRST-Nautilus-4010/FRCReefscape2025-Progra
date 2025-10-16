package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevators.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.commands.Intake;
import frc.robot.subsystems.endEffector.commands.Outake;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
    private final IntakeSubsystem intake = new IntakeSubsystem();
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
    
    private Pose2d getClosestL() {
        //return new Pose2d(11.6, 4.1, Rotation2d.fromDegrees(84.71));
        return new Pose2d(posTracker.getPose().getX() + 0.5, posTracker.getPose().getY(), posTracker.getPose().getRotation());
    }

    private void travel() {
        new ParallelCommandGroup(
            new InstantCommand(() -> setState(RobotState.TRAVEL)),
            shoulder.rotateToTravel(() -> elevator.getHeight()),
            elevator.moveToTravel(() -> shoulder.getAngle())
        ).schedule();
    }

    private void intakeCoral() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> setState(RobotState.INTAKE_CORAL)),
                shoulder.rotateToIntake(() -> elevator.getHeight()),
                elevator.moveToIntake(() -> shoulder.getAngle()),
                intake.goToIntake(),
                new InstantCommand(() -> System.out.println("en primera fase de intake"))
                //swerve.driveTo(getClosestSource())
            ),
            new InstantCommand(() -> System.out.println("empezando segunda fase intake")),
            intake.save()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }

    private void placeL1() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                shoulder.rotateToL1(() -> elevator.getHeight()),
                elevator.moveToL1(() -> shoulder.getAngle()),
                intake.goToL1(),
                posTracker.driveTo(new Pose2d(
                    getClosestSource().getTranslation(),
                    getClosestSource().getRotation().plus(Rotation2d.fromDegrees(-90))
                ))
            ),
            endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }
    
    private void placeL3() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                shoulder.rotateToIntake(() -> elevator.getHeight()),
                elevator.moveToIntake(() -> shoulder.getAngle())
            ),
            new ParallelCommandGroup(
                intake.give(),
                endEffector.intake()
            ),
            new ParallelCommandGroup(
                shoulder.rotateToL3(() -> elevator.getHeight()),
                intake.save()
                //posTracker.driveTo(getClosestL())
            ),
            elevator.moveToL3(() -> shoulder.getAngle()),
            shoulder.placeL3(() -> elevator.getHeight()),
            endEffector.outake()
        ).andThen(() -> scheduleState(RobotState.TRAVEL)).schedule();
    }

    private void placeL4() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                shoulder.rotateToIntake(() -> elevator.getHeight()),
                elevator.moveToIntake(() -> shoulder.getAngle()),
                endEffector.intake()
            ),
            intake.give(),
            new ParallelCommandGroup(
                shoulder.rotateToL4(() -> elevator.getHeight()),
                intake.save(),
                posTracker.driveTo(getClosestL())
            ),
            elevator.moveToL4(() -> shoulder.getAngle()),
            shoulder.placeL4(() -> elevator.getHeight()),
            endEffector.outake()
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