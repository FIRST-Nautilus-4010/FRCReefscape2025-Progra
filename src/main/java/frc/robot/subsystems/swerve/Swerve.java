package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ChassisConstants;

public class Swerve extends SubsystemBase{
    //Defines every single module by giving the drive spark id, the turning spark id, the absolute encoder id, absolute encoder offset, is inverted
    private final SwerveModule frontLeft = new SwerveModule(SwerveConstants.FL_PWR, SwerveConstants.FL_STR, SwerveConstants.FL_ENC);
    private final SwerveModule frontRight = new SwerveModule(SwerveConstants.FR_PWR, SwerveConstants.FR_STR, SwerveConstants.FR_ENC);
    private final SwerveModule backLeft = new SwerveModule(SwerveConstants.BL_PWR, SwerveConstants.BL_STR, SwerveConstants.BL_ENC);
    private final SwerveModule backRight = new SwerveModule(SwerveConstants.BR_PWR, SwerveConstants.BR_STR, SwerveConstants.BR_ENC);

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final Pigeon2 pigeon = new Pigeon2(SwerveConstants.PIGEON);

    private boolean usePigeon = true;
    StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Detected module states", SwerveModuleState.struct).publish();
        
    public Swerve(boolean usePigeon){ 

        zeroHeading(); // Reset the gyroscope When the robot is initialized
        this.usePigeon = usePigeon;
    }

    @Override
    // This repeat periodically during the subsystem use
    public void periodic() {
        swervePublisher.set(getSwerveModuleStates());
        SmartDashboard.putNumber("Robot Heading", getHeading());
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

    // Returns the actual robot angle
    public double getHeading() {
       if (usePigeon) {
           return pigeon.getYaw().getValueAsDouble();
       } else {
           return -gyro.getAngle();
       }
    }
    
    public double getPitch() {
        if (usePigeon) {
            return pigeon.getPitch().getValueAsDouble();
        } else {
            return gyro.getPitch();
        }
    }
    
    public double getRoll() {
        if (usePigeon) {
            return pigeon.getRoll().getValueAsDouble();
        } else {
            return gyro.getRoll();
        }
    }
    
    // Returns a Rotation2d class with the robot angle
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
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

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ChassisConstants.MAX_VELOCITY);
        
        frontLeft.setDesiredState(desiredStates[0], getRoll(), getPitch());
        frontRight.setDesiredState(desiredStates[1], getRoll(), getPitch());
        backLeft.setDesiredState(desiredStates[2], getRoll(), getPitch());
        backRight.setDesiredState(desiredStates[3], getRoll(), getPitch());
    }
}
