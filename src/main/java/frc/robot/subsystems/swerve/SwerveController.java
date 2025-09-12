package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
        var slot1 = configuration.Slot1;
        slot1.kS = SwerveConstants.VEL_KS;
        slot1.kV = SwerveConstants.VEL_KV;
        slot1.kA = SwerveConstants.VEL_KA;
        slot1.kP = SwerveConstants.VEL_KP;
        slot1.kI = SwerveConstants.VEL_KI;
        slot1.kD = SwerveConstants.VEL_KD;
    }

    private void setMMSettings() {
        var motionMagic = configuration.MotionMagic;
        motionMagic.MotionMagicCruiseVelocity = SwerveConstants.MAGIC_MOTION_VELOCITY;
        motionMagic.MotionMagicAcceleration = SwerveConstants.MAGIC_MOTION_ACCELERATION;
        motionMagic.MotionMagicJerk = SwerveConstants.MAGIC_MOTION_JERK;
        motionMagic.MotionMagicExpo_kV = SwerveConstants.MAGIC_MOTION_EXPO_KV; 
        motionMagic.MotionMagicExpo_kA = SwerveConstants.MAGIC_MOTION_EXPO_KA;
    }

    public void setVelocity(double velocity) {
        motor.setControl(velRequest.withVelocity(velocity));
    }

    public double getPID(double currentAngle, double targetAngle) {
        return turningPIDController.calculate(currentAngle, targetAngle);
    }
}
