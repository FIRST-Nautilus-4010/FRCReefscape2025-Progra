package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SaveData;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.HardwareMap;

public class Arm extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(HardwareMap.ARM, MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(HardwareMap.ARM_ENC);
    private final PIDController armPID = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);

    private double armOffset = 0;
    private double pulse2Degree = 0;

    public Arm(boolean armInverted) {
        try {
            armOffset = SaveData.readData("armOffset");
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            pulse2Degree = SaveData.readData("armPulse2Degree");
        } catch (Exception e) {
            e.printStackTrace();
        }

        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.inverted(armInverted);
        armMotor.configure(armMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public double getAngle() {
        return (armEncoder.get() - armOffset) * pulse2Degree;
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
        long startTime = System.currentTimeMillis();
        
        while (true) {
            if (System.currentTimeMillis() - startTime >= 50000) {
                System.out.println("Calibración terminada por tiempo de espera.");
                break;
            }
            
            setArm(-.1);

            if(armMotor.getOutputCurrent() > 30) {
                armOffset = armEncoder.get() - ArmConstants.MIN_ANGLE;
                SaveData.saveData("armOffset", armOffset);
                while (true) {
                    if (System.currentTimeMillis() - startTime >= 50000) {
                        System.out.println("Calibración terminada por tiempo de espera.");
                        break;
                    }
                    
                    setArm(.1);

                    if (armMotor.getOutputCurrent() > 30) {
                        pulse2Degree = ArmConstants.MAX_ANGLE / (armEncoder.get() - armOffset);
                        SaveData.saveData("armPulse2Degree", pulse2Degree);
                        while (getAngle() != 0) {
                            if (System.currentTimeMillis() - startTime >= 50000) {
                                System.out.println("Calibración terminada por tiempo de espera.");
                                break;
                            }

                            setArmPosition(0);
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
}
