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
    private double lastFilteredVel = 0.0; // memoria para el filtro de velocidad
    private static final double VELOCITY_DEADZONE = 0.1; // zona muerta cerca de 0

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState, double chassisRoll, double chassisPitch) {
        Rotation2d encoderRotation = new Rotation2d(io.getAbsoluteEncoderRad());

        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTimestamp;
        if (dt <= 0) dt = 0.02;
        lastTimestamp = currentTime;

        double currentVel = io.getDriveMotorVelocity();

        if (Math.abs(desiredState.speedMetersPerSecond) < VELOCITY_DEADZONE) {
            desiredState.speedMetersPerSecond = 0.0;
        }

        double wantedAcc = (desiredState.speedMetersPerSecond - currentVel) / dt;
        double wantedAngle = desiredState.angle.getRadians();

        double[] limitedAcc = accLimits(wantedAcc, wantedAngle);

        // --- Stability Assist ---
        limitedAcc = applyStabilityAssist(limitedAcc[0], limitedAcc[1], chassisRoll, chassisPitch);
        wantedAcc = limitedAcc[0];

        double nextWantedVel = currentVel + wantedAcc * dt;

        // --- Velocity filter ---
        double deltaVel = nextWantedVel - lastFilteredVel;
        deltaVel = Math.max(Math.min(deltaVel, wantedAcc * dt), -wantedAcc * dt);
        double filteredVel = lastFilteredVel + deltaVel;
        lastFilteredVel = filteredVel;

        desiredState.speedMetersPerSecond = filteredVel;
        desiredState.angle = Rotation2d.fromRadians(limitedAcc[1]);

        desiredState.optimize(encoderRotation);

        controller.setVelocity(desiredState.speedMetersPerSecond);
        controller.setAngle(desiredState.angle.getRadians());

        // --- SmartDashboard debug ---
        if (currentTime - lastDebugTime >= DEBUG_PERIOD) {
            lastDebugTime = currentTime;
            SmartDashboard.putNumber("Wanted Acc " + io.getDriveMotor().getDeviceID(), wantedAcc);
            SmartDashboard.putNumber("Wanted Angle " + io.getDriveMotor().getDeviceID(), wantedAngle);
            SmartDashboard.putNumber("dt " + io.getDriveMotor().getDeviceID(), dt);
            SmartDashboard.putNumber("Filtered Vel " + io.getDriveMotor().getDeviceID(), filteredVel);
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

        double maxSideAccel = SwerveConstants.MAX_SIDE_ACCEL / (massCenterHeight.get() + .001);
        double maxFrontAccel = SwerveConstants.MAX_FRONT_ACCEL / (massCenterHeight.get() + .001);

        finalAccelX = Math.min(finalAccelX, maxSideAccel);
        finalAccelY = Math.min(finalAccelY, maxFrontAccel);

        return new double[] {Math.hypot(finalAccelX, finalAccelY), Math.atan2(finalAccelY, finalAccelX)};
    }

    private double[] applyStabilityAssist(double accHypot, double accAngle, double chassisRoll, double chassisPitch) {
        double maxSafeAngle = 5; // ángulo seguro
        double kAssist = .3; // 0 = domina por completo, 1 = apagado
    
        double accX = accHypot * Math.cos(accAngle);
        double accY = accHypot * Math.sin(accAngle);
    
        double assistX = 0.0;
        double assistY = 0.0;
        
        double maxAccel = accLimits(SwerveConstants.MAX_FORDWARD_ACCEL, accAngle)[0];

        if (Math.abs(chassisPitch) > maxSafeAngle) {
            accX *= kAssist; 
            assistX = -Math.signum(chassisPitch) * ((Math.abs(chassisPitch) - maxSafeAngle) / 80) * (1 - kAssist) * (maxAccel - accHypot) * Math.cos(accAngle);
        }
        if (Math.abs(chassisRoll) > maxSafeAngle) {
            accY *= kAssist;
            assistY = -Math.signum(chassisRoll) * ((Math.abs(chassisRoll) - maxSafeAngle) / 80) * (1 - kAssist) * (maxAccel - accHypot) * Math.sin(accAngle);
        }
    

        accX += assistX;
        accY += assistY;
    
        return new double[] { Math.hypot(accX, accY), Math.atan2(accY, accX) };
    }
}
