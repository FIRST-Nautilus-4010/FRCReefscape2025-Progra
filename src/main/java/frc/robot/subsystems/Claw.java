package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SaveData;
import frc.robot.RobotContainer;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants.ElevatorConstants;
import frc.robot.utils.Constants.HardwareMap;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Claw extends SubsystemBase {
    private final SparkMax rollersMotor = new SparkMax(HardwareMap.ROLLERS, MotorType.kBrushless);
    private final SparkMax rollersMotor1 = new SparkMax(HardwareMap.ROLLERS1, MotorType.kBrushless);

    private final SparkMax clawMotor = new SparkMax(HardwareMap.CLAW, MotorType.kBrushless);
    private final DutyCycleEncoder clawEncoder = new DutyCycleEncoder(HardwareMap.CLAW_ENC);
    private final PIDController clawPID = new PIDController(ClawConstants.CLAW_P, ClawConstants.CLAW_I, ClawConstants.CLAW_D);

    private final SparkMax angleMotor = new SparkMax(HardwareMap.ANGLE, MotorType.kBrushless);
    private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(HardwareMap.ANGLE_ENC);
    private final PIDController anglePID = new PIDController(ClawConstants.ANGLE_P, ClawConstants.ANGLE_I, ClawConstants.ANGLE_D);

    private double clawOffset = 0;
    private double angleOffset = 0;

    private double pulse2Degree = 0;
    private double clawNormalize = 0;
    
    public Claw(boolean rollersInverted, boolean clawInverted, boolean angleInverted) {
        try {
            clawOffset = SaveData.readData("clawOffset");
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            clawNormalize = SaveData.readData("clawNormalize");
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            angleOffset = SaveData.readData("angleOffset");
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            pulse2Degree = SaveData.readData("anglePulse2Degree");
        } catch (Exception e) {
            e.printStackTrace();
        }

        SparkMaxConfig rollersMotorConfig = new SparkMaxConfig();
        rollersMotorConfig.inverted(false);
        rollersMotor.configure(rollersMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        SparkMaxConfig rollersMotor1Config = new SparkMaxConfig();
        rollersMotor1Config.follow(rollersMotor.getDeviceId());
        rollersMotor1.configure(rollersMotor1Config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        SparkMaxConfig clawMotorConfig = new SparkMaxConfig();
        clawMotorConfig.inverted(clawInverted);
        clawMotor.configure(clawMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters   );

        SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
        angleMotorConfig.inverted(angleInverted);
        angleMotor.configure(angleMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }
    
    public double getClawPosition() {
        return clawEncoder.get() / clawNormalize;
    }

    public double getAngle() {
        return angleEncoder.get() * pulse2Degree;
    }

    public void setRollersSpeed(double speed) {
        rollersMotor.set(speed);
    }

    public void setClaw(double speed) {
        if (getClawPosition() >= .97 && speed > 0) {
            clawMotor.set(0);
        } else if (getClawPosition() <= .3 && speed < 0) {
            clawMotor.set(0);
        } else {
            clawMotor.set(speed);
        }
    }

    public void setClawPosition(double position) {
        setClaw(clawPID.calculate(getClawPosition(), position));
    }

    public void setAngle(double velocity) {
        if (getAngle() >= ClawConstants.MAX_ANGLE - 1 && velocity > 0) {
            angleMotor.set(0);
        } else if (getAngle() <= ClawConstants.MIN_ANGLE + 1 && velocity < 0) {
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

    public void stop() {
        set(0, 0, 0);
    }

    public void calibrateClaw() {
        long startTime = System.currentTimeMillis();
        
        while (true) {
            if (System.currentTimeMillis() - startTime >= 50000) {
                System.out.println("Calibración terminada por tiempo de espera.");
                break;
            }
            
            setClaw(-.1);

            if(clawMotor.getOutputCurrent() > 30) {
                clawOffset = clawEncoder.get();
                SaveData.saveData("clawOffset", clawOffset);
                while (true) {
                    if (System.currentTimeMillis() - startTime >= 50000) {
                        System.out.println("Calibración terminada por tiempo de espera.");
                        break;
                    }
                    
                    setClaw(.1);

                    if (clawMotor.getOutputCurrent() > 30) {
                        clawNormalize = 1 / (clawEncoder.get() - clawOffset);
                        SaveData.saveData("clawNormalize", clawNormalize);
                        while (getClawPosition() != 0) {
                            if (System.currentTimeMillis() - startTime >= 50000) {
                                System.out.println("Calibración terminada por tiempo de espera.");
                                break;
                            }

                            setClawPosition(0);
                        }
                        break;
                    }
                }
                break;
            }
        }
    }

    public void calibrateAngle() {
        long startTime = System.currentTimeMillis();
        
        while (true) {
            if (System.currentTimeMillis() - startTime >= 50000) {
                System.out.println("Calibración terminada por tiempo de espera.");
                break;
            }
            
            setAngle(-.1);

            if(angleMotor.getOutputCurrent() > 30) {
                angleOffset = angleEncoder.get() - ClawConstants.MIN_ANGLE;
                SaveData.saveData("angleOffset", angleOffset);
                while (true) {
                    if (System.currentTimeMillis() - startTime >= 50000) {
                        System.out.println("Calibración terminada por tiempo de espera.");
                        break;
                    }
                    
                    setAngle(.1);

                    if (angleMotor.getOutputCurrent() > 30) {
                        pulse2Degree = ClawConstants.MAX_ANGLE / (angleEncoder.get() - angleOffset);
                        SaveData.saveData("anglePulse2Degree", pulse2Degree);
                        while (getAngle() != 0) {
                            if (System.currentTimeMillis() - startTime >= 50000) {
                                System.out.println("Calibración terminada por tiempo de espera.");
                                break;
                            }

                            setAnglePosition(0);
                        }
                        break;
                    }
                }
                break;
            }
        }
    }

    public void calibrate() {
        RobotContainer.elevator.setPosition(ElevatorConstants.MAX_HEIGHT);
        calibrateClaw();
        calibrateAngle();
        RobotContainer.elevator.setPosition(0);
    }
}
