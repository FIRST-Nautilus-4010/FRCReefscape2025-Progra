package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants.HardwareMap;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Claw extends SubsystemBase {
    private final SparkMax rollersMotor = new SparkMax(HardwareMap.ROLLERS, MotorType.kBrushless);

    private final SparkMax clawMotor = new SparkMax(HardwareMap.CLAW, MotorType.kBrushless);
    private final DutyCycleEncoder clawEncoder = new DutyCycleEncoder(HardwareMap.CLAW_ENC);
    private final PIDController clawPID = new PIDController(ClawConstants.CLAW_P, ClawConstants.CLAW_I, ClawConstants.CLAW_D);

    private final SparkMax angleMotor = new SparkMax(HardwareMap.ANGLE, MotorType.kBrushless);
    private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(HardwareMap.ANGLE_ENC);
    private final PIDController anglePID = new PIDController(ClawConstants.ANGLE_P, ClawConstants.ANGLE_I, ClawConstants.ANGLE_D);
    
    public Claw(boolean rollersInverted, boolean clawInverted, boolean angleInverted) {
        SparkMaxConfig rollersMotorConfig = new SparkMaxConfig();
        rollersMotorConfig.inverted(false);
        rollersMotor.configure(rollersMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        SparkMaxConfig clawMotorConfig = new SparkMaxConfig();
        clawMotorConfig.inverted(clawInverted);
        clawMotor.configure(clawMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters   );

        SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
        angleMotorConfig.inverted(angleInverted);
        angleMotor.configure(angleMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }
    
    public double getClawPosition() {
        return clawEncoder.get() / ClawConstants.CLAW_NORMALIZE;
    }

    public double getAngle() {
        return angleEncoder.get() * ClawConstants.PULSE_2_DEGREE;
    }

    public void setRollersSpeed(double speed) {
        rollersMotor.set(speed);
    }

    public void setClaw(double speed) {
        if (getClawPosition() >= 1 && speed > 0) {
            clawMotor.set(0);
        } else if (getClawPosition() <= 0 && speed < 0) {
            clawMotor.set(0);
        } else {
            clawMotor.set(speed);
        }
    }

    public void setClawPosition(double position) {
        setClaw(clawPID.calculate(getClawPosition(), position));
    }

    public void setAngle(double velocity) {
        if (getAngle() >= ClawConstants.MAX_ANGLE && velocity > 0) {
            angleMotor.set(0);
        } else if (getAngle() <= ClawConstants.MIN_ANGLE && velocity < 0) {
            angleMotor.set(0);
        } else {
            angleMotor.set(velocity);
        }
        
    }

    public void setAnglePosition(double angle) {
        setAngle(anglePID.calculate(getAngle(), angle));
    }

    public void set(double rollersSpeed, double clawSpeed, double angleSpeed) {
        setRollersSpeed(rollersSpeed);
        setClaw(clawSpeed);
        setAngle(angleSpeed);
    }

    public void setPosition(double speed, double position, double angle) {
        setRollersSpeed(speed);
        setClawPosition(position);
        setAnglePosition(angle);
    }
}
