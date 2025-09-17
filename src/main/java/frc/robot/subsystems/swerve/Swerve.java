package frc.robot.subsystems.swerve;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Vision;
import frc.robot.utils.Constants.AutonomousConstants;
import frc.robot.utils.Constants.ChassisConstants;

public class Swerve extends SubsystemBase{
    //Defines every single module by giving the drive spark id, the turning spark id, the absolute encoder id, absolute encoder offset, is inverted
    private final SwerveModule frontLeft = new SwerveModule(SwerveConstants.FL_PWR, SwerveConstants.FL_STR, SwerveConstants.FL_ENC);
    private final SwerveModule frontRight = new SwerveModule(SwerveConstants.FR_PWR, SwerveConstants.FR_STR, SwerveConstants.FR_ENC);
    private final SwerveModule backLeft = new SwerveModule(SwerveConstants.BL_PWR, SwerveConstants.BL_STR, SwerveConstants.BL_ENC);
    private final SwerveModule backRight = new SwerveModule(SwerveConstants.BR_PWR, SwerveConstants.BR_STR, SwerveConstants.BR_ENC);

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final Pigeon2 pigeon = new Pigeon2(SwerveConstants.PIGEON);
    Vision vision = new Vision();

    private boolean usePigeon = true;
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Robot position", Pose2d.struct).publish();
    StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Detected module states", SwerveModuleState.struct).publish();
    
    Optional<Alliance> ally = DriverStation.getAlliance();

    TrajectoryConfig trajectoryConfig;
    HolonomicDriveController controller;

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(ChassisConstants.KINEMATICS,
            new Rotation2d(0), getSwerveModulePos(),
            AutonomousConstants.initialPose
    );

    
    public Swerve(boolean usePigeon){ 
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading(); // Reset the gyroscope When the robot is initialized
                ProfiledPIDController thetaController = new ProfiledPIDController(AutonomousConstants.P_Z, AutonomousConstants.I_Z, AutonomousConstants.D_Z, AutonomousConstants.Z_CONTROLER);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                controller = new HolonomicDriveController(
                    new PIDController(AutonomousConstants.P_X, AutonomousConstants.I_X, AutonomousConstants.D_X),
                    new PIDController(AutonomousConstants.P_Y, AutonomousConstants.I_Y, AutonomousConstants.D_Y),
                    thetaController
                );

                this.usePigeon = usePigeon;
                trajectoryConfig = new TrajectoryConfig(AutonomousConstants.MAX_SPD, AutonomousConstants.MAX_ACCEL)
                    .setKinematics(ChassisConstants.KINEMATICS);
            } catch (Exception e) {
            }
        }).start();
    }

    public SwerveModulePosition[] getSwerveModulePos() {
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }
    
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[]{
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    public double getAccelX() {
        if (usePigeon) {
            return pigeon.getAccelerationX().getValue().magnitude();
        } else {
            return gyro.getWorldLinearAccelX();
        }
    }

    public double getAccelY() {
        if (usePigeon) {
            return pigeon.getAccelerationY().getValue().magnitude();
        } else {
            return gyro.getWorldLinearAccelY();
        }
    }

    public double getAccelZ() {
        if (usePigeon) {
            return pigeon.getAccelerationZ().getValue().magnitude();
        } else {
            return gyro.getWorldLinearAccelZ();
        }
    }

    // Reset the gyroscope
    public void zeroHeading() {
        if (usePigeon) {
            pigeon.reset();
        } else {
            gyro.reset();
        }
    }

    // Returns the actual robot angle
    public double getHeading() {
        if (usePigeon) {
            return pigeon.getRotation2d().getDegrees();
        } else {
            return -gyro.getAngle();
        }
    }

    // Returns a Rotation2d class with the robot angle
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
        //return Rotation2d.fromDegrees(36);
    }

    // Return actual robot position
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    // Resets the Odometer
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getSwerveModulePos(), pose);
    }

    // Updates the Odometer
    public void updateOdometry(){
        boolean isRed;
        if (!ally.isEmpty()){
            isRed = ally.get() == Alliance.Red;
        } else {
            isRed = true;
        }

        if (vision.getAprilDetections("limelight-three") > 0){
            resetOdometry(
                vision.getRobotPoseFromAprilTags(
                    getHeading(), 
                    pigeon.getPitch().getValueAsDouble(), 
                    pigeon.getRoll().getValueAsDouble(),
                    isRed
                ).pose
            );
            return;
        }
        odometer.update(getRotation2d(), getSwerveModulePos());
    }

    public Command driveTo(Pose2d pose) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            getPose(),
            List.of(),
            pose,
            trajectoryConfig
        );
        
        return new SwerveControllerCommand(trajectory, this::getPose, ChassisConstants.KINEMATICS, controller, this::setStates, this);
    }

    public Command rotateTo(Rotation2d angle) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            getPose(),
            List.of(),
            new Pose2d(getPose().getTranslation(), angle),
            trajectoryConfig
        );

        return new SwerveControllerCommand(trajectory, this::getPose, ChassisConstants.KINEMATICS, controller, this::setStates, this);
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    @Override
    // This repeat periodically during the subsystem use
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        posePublisher.set(getPose());
        swervePublisher.set(getSwerveModuleStates());
    }


    public void setStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ChassisConstants.MAX_VELOCITY);
        
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
