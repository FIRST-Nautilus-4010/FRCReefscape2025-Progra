package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.Constants.ChassisConstants;



public class SwerveModule {
    final SwerveController controller;
    final SwerveIO io;

    private static Supplier<Double> massCenterHeight = () -> 1.0;

    public SwerveModule(int driveTalonFxId, int turningSparkId, int absoluteEncoderId) {
        io = new SwerveIO(driveTalonFxId, turningSparkId, absoluteEncoderId);
        controller = new SwerveController(io.getDriveMotor());
    }

    public static void setMassCenterHeightSupplier(Supplier<Double> supplier) {
        massCenterHeight = supplier;
    }

    // Returns the current state of the swerve module as a SwerveModuleState object.
    public SwerveModuleState getState() {
        double driveSpeed = io.getDriveMotorVelocity(); // <-- Converts the drive motor speed from RPM to meters per second.
        double turningPosition = io.getAbsoluteEncoderRad(); // <-- Retrieves the current turning position in radians.

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition)); // <-- Creates a SwerveModuleState with speed and rotation.
    }

    // Returns the current position of the swerve module as a SwerveModulePosition object.
    public SwerveModulePosition getPosition() {
        double driveDistance = io.getDriveMotorPosition(); // <-- Converts the drive motor encoder rotations to meters traveled.
        double turningPosition = io.getAbsoluteEncoderRad() * SwerveConstants.ROT_2_RAD; // <-- Converts the turning encoder rotations to radians.

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition)); // <-- Creates a SwerveModulePosition with distance and rotation.
    }

    public void stop() {
        io.stop(); // <-- Stops both the drive and turning motors.
    }

    public double accLimits(double wantedAcc, double angle) {
        double dirX = Math.cos(angle);
        double dirY = Math.sin(angle);
    
        double safeDirX = (dirX == 0) ? 1e-9 : dirX;
        double safeDirY = (dirY == 0) ? 1e-9 : dirY;
    
        double maxAccX = (SwerveConstants.MAX_SIDE_ACCEL / massCenterHeight.get()) / Math.abs(safeDirX);
        double maxAccY = (SwerveConstants.MAX_FRONT_ACCEL / massCenterHeight.get()) / Math.abs(safeDirY);
    
        double directionalLimit = Math.min(maxAccX, Math.min(maxAccY, wantedAcc));
    
        double fordwardAccel = SwerveConstants.MAX_FORDWARD_ACCEL * (1 - (io.getDriveMotorVelocity() / ChassisConstants.MAX_VELOCITY));
    
        double skidAccel = Math.min(SwerveConstants.MAX_SKID_ACCEL, wantedAcc);
    
        double finalAccel = Math.min(directionalLimit, Math.min(fordwardAccel, skidAccel));
    
        return finalAccel;
    }

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState) {

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.09) {
            io.stop(); // <-- Stops both the drive and turning motors.
            return; // <-- Exits the method to prevent unnecessary movement.
        }

        Rotation2d encoderRotation = new Rotation2d(io.getAbsoluteEncoderRad());

        desiredState.optimize(encoderRotation);
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos();
        
        double wantedAcc = (desiredState.speedMetersPerSecond - io.getDriveMotorVelocity()) / 0.02; // <-- Calculates the required acceleration to reach the desired speed in one control loop cycle (20ms).
        wantedAcc = accLimits(wantedAcc, desiredState.angle.getRadians());

        controller.setMMAccel(wantedAcc);
        
        double nextWantedVel = io.getDriveMotorVelocity() + wantedAcc * 0.02;
        controller.setVelocity(nextWantedVel);

        double output = controller.getPID(io.getAbsoluteEncoderRad(), desiredState.angle.getRadians());
        io.getTurningMotor().set(output);
    }
}