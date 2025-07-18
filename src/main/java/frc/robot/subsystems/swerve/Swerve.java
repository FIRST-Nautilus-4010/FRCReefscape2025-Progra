package frc.robot.subsystems.swerve;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Vision;
import frc.robot.utils.Constants.AutonomousConstants;
import frc.robot.utils.Constants.ChassisConstants;

public class Swerve extends SubsystemBase{
    //--------Constants--------

    //--------Rev Robotics--------

    // SPARKS
    public static final int FL_STR = 1;
    public static final int FR_STR = 2;
    public static final int BL_STR = 3;
    public static final int BR_STR = 4;

    public static final int ELEVATOR = 5;
    public static final int ELEVATOR1 = 6;

    public static final int ROLLERS = 7;
    public static final int ROLLERS1 = 8;
    public static final int CLAW = 9;
    public static final int ANGLE = 10;

    public static final int ARM = 11;
    public static final int ARM1 = 12;

    //--------CTR Electronics--------

    // Krakens 
    public static final int FL_PWR = 1;
    public static final int FR_PWR = 2;
    public static final int BL_PWR = 3;
    public static final int BR_PWR = 4;

    // Swerve encoders
    public static final int FL_ENC = 5;
    public static final int FR_ENC = 6;
    public static final int BL_ENC = 7;
    public static final int BR_ENC = 8;

    // Gyro
    public static final int PIGEON = 9;

    //------------roboRIO--------------

    // DIO
    public static final int ELEVATOR_ENC = 0;
    public static final int BOTTOM_LIMIT = 1;
    public static final int TOP_LIMIT = 2;

    public static final int CLAW_ENC = 3;
    public static final int ANGLE_ENC = 4;

    public static final int ARM_ENC = 5;

    // Encoder offsets
    public static final double[] ENCODER_OFFSETS = {0, 0, 0, 0}; // <-- {FL, FR, BL, BR} offsets.

    //Defines every single module by giving the drive spark id, the turning spark id, the absolute encoder id, absolute encoder offset, is inverted
    private final SwerveModule frontLeft = new SwerveModule(FL_PWR, FL_STR, FL_ENC, ENCODER_OFFSETS[0], false, false);
    private final SwerveModule frontRight = new SwerveModule(FR_PWR, FR_STR, FR_ENC, ENCODER_OFFSETS[1], false, false);
    private final SwerveModule backLeft = new SwerveModule(BL_PWR, BL_STR, BL_ENC, ENCODER_OFFSETS[2], false, false);
    private final SwerveModule backRight = new SwerveModule(BR_PWR, BR_STR, BR_ENC, ENCODER_OFFSETS[3], false, false);

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final Pigeon2 pigeon = new Pigeon2(PIGEON);
    Vision vision = new Vision();

    private boolean usePigeon = true;
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Robot position", Pose2d.struct).publish();
    StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Detected module states", SwerveModuleState.struct).publish();
    
    Optional<Alliance> ally = DriverStation.getAlliance();

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(ChassisConstants.KINEMATICS,
            new Rotation2d(0), getSwerveModulePos(),
            AutonomousConstants.initialPose
    );

    
    public Swerve(boolean usePigeon){ 
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading(); // Reset the gyroscope When the robot is initialized
            } catch (Exception e) {
            }
        }).start();

        this.usePigeon = usePigeon;
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

    @Override
    // This repeat periodically during the subsystem use
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        posePublisher.set(getPose());
        swervePublisher.set(getSwerveModuleStates());
    }

    // Stop the swerve modules
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ChassisConstants.MAX_SPD);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
