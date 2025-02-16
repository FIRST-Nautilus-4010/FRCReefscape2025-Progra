package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public double getAmp() {
        return elevatorMotor.getOutputCurrent();
    }

    public boolean getBottomLimitSwitch() {
        return bottomLimitSwitch.get();
    }
    
    public boolean getTopLimitSwitch() {
        return topLimitSwitch.get();
    }

    public double getEncoderPos() {
        return (elevatorEncoder.get() - elevatorOffset) * pulse2M;
    }

    public void set(double speed) {
        if (speed > 0 && (getTopLimitSwitch() || getEncoderPos() > ElevatorConstants.MAX_HEIGHT)) {
            stop();
            pulse2M = ElevatorConstants.MAX_HEIGHT / (elevatorEncoder.get() - elevatorOffset);
            SaveData.saveData("pulse2M", pulse2M);
        } else if (speed < 0 && (getBottomLimitSwitch() || getEncoderPos() < 0)) {
            stop();
            elevatorOffset = elevatorEncoder.get();
            SaveData.saveData("elevatorOffset", elevatorOffset);
        } else {
            elevatorMotor.set(speed);
        }
    }

    public void setPos(double position) {
        elevatorMotor.set(elevatorPID.calculate(getEncoderPos(), position));
    }

    public void stop() {
        elevatorMotor.stopMotor();
    }

    public void calibrate() {
        long startTime = System.currentTimeMillis();
        
        while (true) {
            if (System.currentTimeMillis() - startTime >= 50000) {
                System.out.println("Calibración terminada por tiempo de espera.");
                break;
            }
            
            set(-.1);

            if(getBottomLimitSwitch() || getAmp() > ElevatorConstants.AMP_THRESHOLD) {
                while (true) {
                    if (System.currentTimeMillis() - startTime >= 50000) {
                        System.out.println("Calibración terminada por tiempo de espera.");
                        break;
                    }
                    
                    set(.1);

                    if (getTopLimitSwitch() || getAmp() > ElevatorConstants.AMP_THRESHOLD) {
                        while (getEncoderPos() != 0) {
                            if (System.currentTimeMillis() - startTime >= 50000) {
                                System.out.println("Calibración terminada por tiempo de espera.");
                                break;
                            }

                            setPos(0);
                        }
                        break;
                    }
                }
                break;
            }
        }
    }

    public void update() {
        SmartDashboard.putNumber("Elevator Position", getEncoderPos());
        SmartDashboard.putNumber("Elevator amp", getAmp());
    }

}