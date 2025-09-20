package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ChassisConstants;

public class SwerveModule {
    final SwerveController controller;
    final SwerveIO io;

    private static Supplier<Double> massCenterHeight = () -> 1.0;

    public SwerveModule(int driveTalonFxId, int turningSparkId, int absoluteEncoderId) {
        io = new SwerveIO(driveTalonFxId, turningSparkId, absoluteEncoderId);
        controller = new SwerveController(io.getDriveMotor(), io.getTurningMotor());
    }

    public static void setMassCenterHeightSupplier(Supplier<Double> supplier) {
        massCenterHeight = supplier;
    }

    // Returns the current state of the swerve module as a SwerveModuleState object.
    public SwerveModuleState getState() {
        double driveSpeed = io.getDriveMotorVelocity(); 
        double turningPosition = io.getAbsoluteEncoderRad(); 

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition));
    }

    // Returns the current position of the swerve module as a SwerveModulePosition object.
    public SwerveModulePosition getPosition() {
        double driveDistance = io.getDriveMotorPosition(); 
        double turningPosition = io.getAbsoluteEncoderRad(); 

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition));
    }

    public void stop() {
        io.stop(); 
    }
    
    private double lastDebugTime = 0.0;
    private static final double DEBUG_PERIOD = 1;

    private double lastTimestamp = Timer.getFPGATimestamp();

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d encoderRotation = new Rotation2d(io.getAbsoluteEncoderRad());

        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTimestamp;
        if (dt <= 0) dt = 0.02;
        lastTimestamp = currentTime;

        desiredState.optimize(encoderRotation);
        
        double currentVel = io.getDriveMotorVelocity();
        double wantedAcc = (desiredState.speedMetersPerSecond - currentVel) / dt;
        double wantedAngle = desiredState.angle.getRadians();

        double[] limitedAcc = accLimits(wantedAcc, wantedAngle);

        wantedAcc = limitedAcc[0];
        wantedAngle = limitedAcc[1];

        double nextWantedVel = currentVel + wantedAcc * dt;

        controller.setVelocity(nextWantedVel);
        controller.setAngle(wantedAngle);

        if (currentTime - lastDebugTime >= DEBUG_PERIOD) {
            lastDebugTime = currentTime;
    
            SmartDashboard.putNumber("Wanted Acc " + io.getDriveMotor().getDeviceID(), wantedAcc);
            SmartDashboard.putNumber("Wanted Angle " + io.getDriveMotor().getDeviceID(), wantedAngle);
            SmartDashboard.putNumber("dt " + io.getDriveMotor().getDeviceID(), dt);
        }

    }

    private double[] accLimits(double accHypot, double accAngle) {
        double desiredAccelX = accHypot * Math.cos(accAngle);
        double desiredAccelY = accHypot * Math.sin(accAngle);

        double accelModule = Math.hypot(desiredAccelX, desiredAccelY);

        double forwardAccel = Math.min(accelModule, SwerveConstants.MAX_FORDWARD_ACCEL * (1 - (io.getDriveMotorVelocity() / ChassisConstants.MAX_VELOCITY)));
        double skidAccel = Math.min(accelModule, SwerveConstants.MAX_SKID_ACCEL);
        
        if (accelModule < 1e-6) return new double[] {0.0, 0.0};

        double finalAccelX = (Math.abs(desiredAccelX) / accelModule) * Math.min(skidAccel, forwardAccel);
        double finalAccelY = (Math.abs(desiredAccelY) / accelModule) * Math.min(skidAccel, forwardAccel);

        finalAccelX = Math.min(finalAccelX, SwerveConstants.MAX_SIDE_ACCEL / massCenterHeight.get()) * Math.signum(desiredAccelX);
        finalAccelY = Math.min(finalAccelY, SwerveConstants.MAX_FRONT_ACCEL / massCenterHeight.get()) * Math.signum(desiredAccelY);

        return new double[] {Math.hypot(finalAccelX, finalAccelY), Math.atan2(finalAccelY, finalAccelX)};
    }
}
