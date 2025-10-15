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

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState, double chassisRoll, double chassisPitch) {
        Rotation2d encoderRotation = new Rotation2d(io.getAbsoluteEncoderRad());

        double desiredFinalVel = desiredState.speedMetersPerSecond;
        double currentVel = Math.abs(io.getDriveMotorVelocity());

        double wantedDirection = desiredState.angle.getRadians();

        double wantedAcc = (desiredFinalVel - currentVel) / 0.02;
        double[] accLimits = accLimits(wantedAcc, wantedDirection);
        //accLimits = applyStabilityAssist(accLimits[0], accLimits[1], chassisRoll, chassisPitch);

        double limitedAcc = accLimits[0];
        double limitedDirection = accLimits[1];

        double nextWantedVel = currentVel + (limitedAcc * 0.02);

        SwerveModuleState optimizedState = new SwerveModuleState(nextWantedVel, Rotation2d.fromRadians(limitedDirection));

        optimizedState.optimize(encoderRotation);

        if (Math.abs(desiredState.speedMetersPerSecond) < SwerveConstants.VELOCITY_DEADZONE) {
            controller.setVelocity(0);
        } else {
            controller.setVelocity(optimizedState.speedMetersPerSecond);
        }
        controller.setAngle(optimizedState.angle.getRadians());

        // --- SmartDashboard debug ---
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastDebugTime >= 1) {
            lastDebugTime = currentTime;
            SmartDashboard.putNumber("Wanted Acc " + io.getDriveMotor().getDeviceID(), wantedAcc);
            SmartDashboard.putNumber("Limited Acc " + io.getDriveMotor().getDeviceID(), limitedAcc);

            SmartDashboard.putNumber("Wanted Side Acc " + io.getDriveMotor().getDeviceID(), wantedAcc * Math.cos(wantedDirection));
            SmartDashboard.putNumber("Wanted Front Acc " + io.getDriveMotor().getDeviceID(), wantedAcc * Math.sin(wantedDirection));

            SmartDashboard.putNumber("Limited Side Acc " + io.getDriveMotor().getDeviceID(), limitedAcc * Math.cos(limitedDirection));
            SmartDashboard.putNumber("Limited Front Acc " + io.getDriveMotor().getDeviceID(), limitedAcc * Math.sin(limitedDirection));

            SmartDashboard.putNumber("Wanted Direction " + io.getDriveMotor().getDeviceID(), wantedDirection);
            SmartDashboard.putNumber("Limited Direction " + io.getDriveMotor().getDeviceID(), limitedDirection);

            SmartDashboard.putNumber("Desired Final Vel " + io.getDriveMotor().getDeviceID(), desiredFinalVel);
            SmartDashboard.putNumber("Current Vel " + io.getDriveMotor().getDeviceID(), currentVel);
            SmartDashboard.putNumber("Next Wanted Vel " + io.getDriveMotor().getDeviceID(), nextWantedVel);
        }
    }

    

    private double[] accLimits(double wantedAcc, double wantedDirection) {
        double wantedAccMagnitude = Math.abs(wantedAcc);

        double forwardAccel = Math.min(wantedAccMagnitude, SwerveConstants.MAX_FORDWARD_ACCEL * (1 - (io.getDriveMotorVelocity() / ChassisConstants.MAX_VELOCITY)));
        double skidAccel = Math.min(wantedAccMagnitude, SwerveConstants.MAX_SKID_ACCEL);

        double minAccel = Math.min(skidAccel, forwardAccel);

        // Controles tilt
        double wantedSideAcc = minAccel * -Math.sin(wantedDirection);
        double wantedFrontAcc = minAccel * Math.cos(wantedDirection);
        
        double limitedFrontAcc = Math.max(-SwerveConstants.MAX_FRONT_ACCEL - massCenterHeight.get() * SwerveConstants.MAX_FRONT_ACCEL_MAX / SwerveConstants.MAX_HEIGHT, Math.min(wantedFrontAcc, SwerveConstants.MAX_FRONT_ACCEL));
        double limitedSideAcc = Math.max(-SwerveConstants.MAX_SIDE_ACCEL - massCenterHeight.get() * SwerveConstants.MAX_SIDE_ACCEL_MAX / SwerveConstants.MAX_HEIGHT, Math.min(wantedSideAcc, SwerveConstants.MAX_SIDE_ACCEL));

        double limitedAcc = Math.hypot(limitedFrontAcc, limitedSideAcc);

        double limitedDirection;
        if (Math.abs(limitedSideAcc) < SwerveConstants.MAX_SIDE_ACCEL && Math.abs(limitedFrontAcc) < SwerveConstants.MAX_SIDE_ACCEL) {
            limitedDirection = wantedDirection;
        } else {
            limitedDirection = Math.atan2(-limitedSideAcc, limitedFrontAcc);
        }
        
        
        //return new double[] {Math.copySign(minAccel, wantedAcc), wantedDirection};
        return new double[] {Math.copySign(limitedAcc, wantedAcc), limitedDirection};
    }

    private double[] applyStabilityAssist(double accHypot, double accAngle, double chassisRoll, double chassisPitch) {
        double maxSafeAngle = 5; // ángulo seguro
        double kAssist = .3; // 0 = domina por completo, 1 = apagado
    
        double accX = accHypot * Math.cos(accAngle);
        double accY = accHypot * Math.sin(accAngle);
    
        double assistX = 0.0;
        double assistY = 0.0;
        
        if (Math.abs(chassisPitch) > maxSafeAngle) {
            accX *= kAssist; 
            assistX = -Math.signum(chassisPitch) * 
                ((Math.abs(chassisPitch) - maxSafeAngle) / 80) * 
                (1 - kAssist) * 
                (SwerveConstants.MAX_SKID_ACCEL - accHypot) * 
                Math.cos(accAngle);
        }
        if (Math.abs(chassisRoll) > maxSafeAngle) {
            accY *= kAssist;
            assistY = -Math.signum(chassisRoll) * 
                ((Math.abs(chassisRoll) - maxSafeAngle) / 80) * 
                (1 - kAssist) * 
                (SwerveConstants.MAX_SKID_ACCEL - accHypot) * 
                Math.sin(accAngle);
        }
    

        accX += assistX;
        accY += assistY;
    
        return new double[] { Math.hypot(accX, accY), Math.atan2(accY, accX) };
    }
}
