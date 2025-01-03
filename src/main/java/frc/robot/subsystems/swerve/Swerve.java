package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Swerve extends SubsystemBase{

    private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

    //Defines every single module by giving the drive spark id, the turning spark id, the absolute encoder id, absolute encoder offset, is inverted
    private final SwerveModule frontLeft = new SwerveModule(Constants.HardwareMap.FL_PWR, Constants.HardwareMap.FL_STR, Constants.HardwareMap.FL_ENC, Constants.ModuleConstants.ENCODER_OFFSETS[0], false, false);
    private final SwerveModule frontRight = new SwerveModule(Constants.HardwareMap.FR_PWR, Constants.HardwareMap.FR_STR, Constants.HardwareMap.FR_ENC, Constants.ModuleConstants.ENCODER_OFFSETS[1], true, false);
    private final SwerveModule backLeft = new SwerveModule(Constants.HardwareMap.BL_PWR, Constants.HardwareMap.BL_STR, Constants.HardwareMap.BL_ENC, Constants.ModuleConstants.ENCODER_OFFSETS[2], true, false);
    private final SwerveModule backRight = new SwerveModule(Constants.HardwareMap.BR_PWR, Constants.HardwareMap.BR_STR, Constants.HardwareMap.BR_ENC,  Constants.ModuleConstants.ENCODER_OFFSETS[3], false, false);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public int robot_turning_encoder = 0;

    // Convert to module states
    public SwerveModuleState[] moduleStates;

    private final SwerveModulePosition[] swerve_module_position = new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
        };
    
    public final SwerveModuleState[] swerve_module_states = new SwerveModuleState[]{
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
        };
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.ChassisConstants.KINEMATICS,
            new Rotation2d(0), swerve_module_position,
            Constants.AutonomousConstants.initialPose);

    
    public Swerve() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading(); // Reset the gyroscope When the robot is initialized
            } catch (Exception e) {
            }
        }).start();
    }

    // Reset the gyroscope
    public void zeroHeading() {
        gyro.reset();
    }

    // Returns the actual robot angle
    public double getHeading() {
        return (-gyro.getAngle());
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
        odometer.resetPosition(getRotation2d(), swerve_module_position, pose);
    }

    // Updates the Odometer
    public void updateOdometry(){
        updateModulePosition();
        odometer.update(getRotation2d(), swerve_module_position);
    }
    
    // Update the module states reading
    public void updateModuleStates(){
        swerve_module_states[0] = frontLeft.getState();
        swerve_module_states[1] = frontRight.getState();
        swerve_module_states[2] = backLeft.getState();
        swerve_module_states[3] = backRight.getState();
    }
    public void updateModulePosition(){
        swerve_module_position[0] = frontLeft.getPosition();
        swerve_module_position[1] = frontRight.getPosition();
        swerve_module_position[2] = backLeft.getPosition();
        swerve_module_position[3] = backRight.getPosition();
    }

    @Override
    // This repeat periodically during the subsystem use
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    // Stop the swerve modules
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Set the desired state for each swerveModule by giving an array of states
    public void setStates(SwerveModuleState[] desired_states, ChassisSpeeds chassisSpeeds){
        speeds = chassisSpeeds;

        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, Constants.ChassisConstants.MAX_SPD);
        frontLeft.setDesiredState(desired_states[0]);
        frontRight.setDesiredState(desired_states[1]);
        backLeft.setDesiredState(desired_states[2]);
        backRight.setDesiredState(desired_states[3]);
    }

    public void setStatesWithoutSpeeds(SwerveModuleState[] desired_states){
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, Constants.ChassisConstants.MAX_SPD);
        frontLeft.setDesiredState(desired_states[0]);
        frontRight.setDesiredState(desired_states[1]);
        backLeft.setDesiredState(desired_states[2]);
        backRight.setDesiredState(desired_states[3]);
    }

    public ChassisSpeeds getSpeeds(){
        return speeds;
    }
}
