package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SaveData;
import frc.robot.utils.Constants.ElevatorConstants;
import frc.robot.utils.Constants.HardwareMap;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor = new SparkMax(HardwareMap.ELEVATOR, MotorType.kBrushless);
    private final SparkMax elevatorMotor1 = new SparkMax(HardwareMap.ELEVATOR1, MotorType.kBrushless);
    private final DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(HardwareMap.ELEVATOR_ENC);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(HardwareMap.BOTTOM_LIMIT);
    private final DigitalInput topLimitSwitch = new DigitalInput(HardwareMap.TOP_LIMIT);
    private final PIDController elevatorPID = new PIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D);

    private double elevatorOffset = 0;
    private double pulse2M = 0;

    public Elevator(boolean isInverted) {
        try {
            elevatorOffset = SaveData.readData("elevatorOffset");
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            pulse2M = SaveData.readData("pulse2M");
        } catch (Exception e) {
            e.printStackTrace();
        }

        SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
        elevatorMotorConfig.inverted(isInverted);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig elevatorMotor1Config = new SparkMaxConfig();
        elevatorMotor1Config.follow(elevatorMotor.getDeviceId());
        elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean getBottomLimitSwitch() {
        return bottomLimitSwitch.get();
    }
    
    public boolean getTopLimitSwitch() {
        return topLimitSwitch.get();
    }

    public double getEncoderPosition() {
        return (elevatorEncoder.get() - elevatorOffset) * pulse2M;
    }

    public void setSpeed(double speed) {
        if (speed > 0 && getTopLimitSwitch()) {
            stop();
            pulse2M = ElevatorConstants.MAX_HEIGHT / (elevatorEncoder.get() - elevatorOffset);
            SaveData.saveData("pulse2M", pulse2M);
        } else if (speed < 0 && getBottomLimitSwitch()) {
            stop();
            elevatorOffset = elevatorEncoder.get();
            SaveData.saveData("elevatorOffset", elevatorOffset);
        } else {
            elevatorMotor.set(speed);
        }
    }

    public void setPosition(double position) {
        elevatorMotor.set(elevatorPID.calculate(getEncoderPosition(), position));
    }

    public void stop() {
        elevatorMotor.set(0);
    }

    public void calibrate() {
        long startTime = System.currentTimeMillis();
        
        while (true) {
            if (System.currentTimeMillis() - startTime >= 50000) {
                System.out.println("Calibración terminada por tiempo de espera.");
                break;
            }
            
            setSpeed(-.1);

            if(getBottomLimitSwitch()) {
                while (true) {
                    if (System.currentTimeMillis() - startTime >= 50000) {
                        System.out.println("Calibración terminada por tiempo de espera.");
                        break;
                    }
                    
                    setSpeed(.1);

                    if (getTopLimitSwitch()) {
                        while (getEncoderPosition() != 0) {
                            if (System.currentTimeMillis() - startTime >= 50000) {
                                System.out.println("Calibración terminada por tiempo de espera.");
                                break;
                            }

                            setPosition(0);
                        }
                        break;
                    }
                }
                break;
            }
        }
    }

}