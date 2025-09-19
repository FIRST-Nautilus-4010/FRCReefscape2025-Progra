package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveController {
    private final TalonFX motor;
    private final TalonFXConfiguration configuration;
    private final MotionMagicVelocityVoltage velRequest;
    private final ProfiledPIDController turningPIDController;

    public SwerveController(TalonFX motor) {
        this.motor = motor;

        velRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

        configuration = new TalonFXConfiguration();

        setVelocitySlotGains();
        setMMSettings();

        motor.getConfigurator().apply(configuration);

        // Assigns a pid controller for the turning motor. This one takes a P variable stablish in constants that specifies the proportional PID value
        turningPIDController = new ProfiledPIDController(
            SwerveConstants.PID_P, 
            SwerveConstants.PID_I, 
            SwerveConstants.PID_D, 
            new TrapezoidProfile.Constraints(
                6.7 * 2 * Math.PI, // <-- Maximum angular velocity in radians per second.
                10000 // <-- Maximum angular acceleration in radians per second squared.
            )
        );
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void setVelocitySlotGains() {
        var slot0 = configuration.Slot0;
        slot0.kS = SwerveConstants.VEL_KS;
        slot0.kV = SwerveConstants.VEL_KV;
        slot0.kA = SwerveConstants.VEL_KA;
        slot0.kP = SwerveConstants.VEL_KP;
        slot0.kI = SwerveConstants.VEL_KI;
        slot0.kD = SwerveConstants.VEL_KD;
    }

    private void setMMSettings() {
        var motionMagic = configuration.MotionMagic;
        motionMagic.MotionMagicJerk = SwerveConstants.MAGIC_MOTION_JERK;
    }

    public void setMMAccel(double accel) {
        SmartDashboard.putNumber("KrakenSetAcceleration", Math.abs(accel));
        velRequest.Acceleration = Math.abs(accel) / (SwerveConstants.ROT_2_M * SwerveConstants.ROT_2_M);
    }

    public void setVelocity(double velocity) {
        double adjustedVelocity = velocity / SwerveConstants.ROT_2_M;
        motor.setControl(velRequest.withVelocity(adjustedVelocity));
    }

    public double getPID(double currentAngle, double targetAngle) {
        return turningPIDController.calculate(currentAngle, targetAngle);
    }
}
