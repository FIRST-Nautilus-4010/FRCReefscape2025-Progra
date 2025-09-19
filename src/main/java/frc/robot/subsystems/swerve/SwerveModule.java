package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        double driveSpeed = io.getDriveMotorVelocity(); 
        double turningPosition = io.getAbsoluteEncoderRad(); 

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition));
    }

    // Returns the current position of the swerve module as a SwerveModulePosition object.
    public SwerveModulePosition getPosition() {
        double driveDistance = io.getDriveMotorPosition(); 
        double turningPosition = io.getAbsoluteEncoderRad() * SwerveConstants.ROT_2_RAD; 

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition));
    }

    public void stop() {
        io.stop(); 
    }

    public double accLimits(double wantedAcc, double angle) {
        double dirX = Math.cos(angle);
        double dirY = Math.sin(angle);
    
        double safeDirX = (Math.abs(dirX) < 1e-6) ? 1e-6 : dirX;
        double safeDirY = (Math.abs(dirY) < 1e-6) ? 1e-6 : dirY;
    
        // Proteger altura del centro de masa
        double massH = Math.max(1e-6, massCenterHeight.get());
    
        // Límites como magnitud
        double maxAccX = (SwerveConstants.MAX_SIDE_ACCEL / massH) / Math.abs(safeDirX);
        double maxAccY = (SwerveConstants.MAX_FRONT_ACCEL / massH) / Math.abs(safeDirY);
    
        // Proteger contra NaN o Infinity
        if (Double.isNaN(maxAccX) || Double.isInfinite(maxAccX)) maxAccX = 1e6;
        if (Double.isNaN(maxAccY) || Double.isInfinite(maxAccY)) maxAccY = 1e6;
    
        double wantedAccAbs = Math.abs(wantedAcc);
        double directionalLimit = Math.min(maxAccX, maxAccY);
    
        double fordwardAccel = SwerveConstants.MAX_FORDWARD_ACCEL *
                Math.max(0.0, 1 - (io.getDriveMotorVelocity() / ChassisConstants.MAX_VELOCITY));
    
        double skidAccel = SwerveConstants.MAX_SKID_ACCEL;
    
        double finalAccelAbs = wantedAccAbs;
        finalAccelAbs = Math.min(finalAccelAbs, directionalLimit);
        finalAccelAbs = Math.min(finalAccelAbs, fordwardAccel);
        finalAccelAbs = Math.min(finalAccelAbs, skidAccel);
    
        SmartDashboard.putNumber("Limit Dir", directionalLimit);
        SmartDashboard.putNumber("Limit Fwd", fordwardAccel);
        SmartDashboard.putNumber("Limit Skid", skidAccel);
        SmartDashboard.putNumber("Final Acc Abs", finalAccelAbs);
    
        return Math.copySign(finalAccelAbs, wantedAcc);
    }
    

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d encoderRotation = new Rotation2d(io.getAbsoluteEncoderRad());

        desiredState.optimize(encoderRotation);
        
        double currentVel = io.getDriveMotorVelocity();
        double wantedAcc = (desiredState.speedMetersPerSecond - currentVel) / 0.02;

        // Aplicar límites
        wantedAcc = accLimits(wantedAcc, desiredState.angle.getRadians());

        controller.setMMAccel(wantedAcc);

        // Calcular la próxima velocidad deseada
        double nextWantedVel = currentVel + wantedAcc * 0.02;

        controller.setVelocity(nextWantedVel);

        // Controlar el ángulo con el PID
        double output = controller.getPID(io.getAbsoluteEncoderRad(), desiredState.angle.getRadians());
        io.getTurningMotor().set(output);

        // Mostrar datos en SmartDashboard
        SmartDashboard.putNumber("Desired Velocity", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Wanted Acc", wantedAcc);
        SmartDashboard.putNumber("Wanted Vel", nextWantedVel);
        SmartDashboard.putNumber("Real Vel", currentVel);
    }
}
