package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.HardwareMap;

public class Arm extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(HardwareMap.ARM, MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(HardwareMap.ARM_ENC);
    private final PIDController armPID = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);

    private double armOffset = 0;
    private double pulse2Degree = 0;

    public Arm(boolean armInverted) {
        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.inverted(armInverted);
        armMotor.configure(armMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public double getAngle() {
        return armEncoder.get() * pulse2Degree;
    }

    public void setArm(double speed) {
        if (getAngle() >= ArmConstants.MAX_ANGLE && speed > 0) {
            armMotor.set(0);
        } else if (getAngle() <= ArmConstants.MIN_ANGLE && speed < 0) {
            armMotor.set(0);
        } else {
            armMotor.set(speed);
        }
    }

    public void setArmPosition(double angle) {
        setArm(armPID.calculate(getAngle(), angle));
    }

    public void calibrate() {
        
    }
}
