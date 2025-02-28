package frc.robot.subsystems.swerve;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants.HardwareMap;
import frc.robot.RobotContainer;
import frc.robot.utils.Constants.AutonomousConstants;
import frc.robot.utils.Constants.ChassisConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.VisionConstants;

public class Swerve extends SubsystemBase{

    //Defines every single module by giving the drive spark id, the turning spark id, the absolute encoder id, absolute encoder offset, is inverted
    private final SwerveModule frontLeft = new SwerveModule(HardwareMap.FL_PWR, HardwareMap.FL_STR, HardwareMap.FL_ENC, ModuleConstants.ENCODER_OFFSETS[0], false, false);
    private final SwerveModule frontRight = new SwerveModule(HardwareMap.FR_PWR, HardwareMap.FR_STR, HardwareMap.FR_ENC, ModuleConstants.ENCODER_OFFSETS[1], false, false);
    private final SwerveModule backLeft = new SwerveModule(HardwareMap.BL_PWR, HardwareMap.BL_STR, HardwareMap.BL_ENC, ModuleConstants.ENCODER_OFFSETS[2], false, false);
    private final SwerveModule backRight = new SwerveModule(HardwareMap.BR_PWR, HardwareMap.BR_STR, HardwareMap.BR_ENC, ModuleConstants.ENCODER_OFFSETS[3], false, false);

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final Pigeon2 pigeon = new Pigeon2(HardwareMap.PIGEON);

    private boolean usePigeon = true;

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
        for (int i = 0; i < VisionConstants.CAM_NUM; i++){
            if (RobotContainer.vision.getAprilDetections(i) > 0) {
                double[] botpose = RobotContainer.vision.getRobotPoseFromFirstAprilTag(i);
                Pose2d pose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[2]));
                odometer.resetPose(pose);
                return;
            }

        }
        odometer.update(getRotation2d(), getSwerveModulePos());
    }

    @Override
    // This repeat periodically during the subsystem use
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        NetworkTableInstance.getDefault().getStructTopic("Robot position", Pose2d.struct).publish().set(getPose());
        NetworkTableInstance.getDefault().getStructArrayTopic("Detected module states", SwerveModuleState.struct).publish().set(getSwerveModuleStates());
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
