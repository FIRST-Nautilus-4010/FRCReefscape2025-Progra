package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SaveData;
import frc.robot.RobotContainer;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.ElevatorConstants;
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

    public void set(double speed) {
        if (getAngle() >= ArmConstants.MAX_ANGLE - 1 && speed > 0) {
            armMotor.set(0);
        } else if (getAngle() <= ArmConstants.MIN_ANGLE + 1 && speed < 0) {
            armMotor.set(0);
        } else {
            armMotor.set(speed);
        }
    }

    public void setPos(double angle) {
        set(armPID.calculate(getAngle(), angle));
    }

    public double getAmp() {
        return armMotor.getOutputCurrent();
    }

    public void calibrate() {
        RobotContainer.elevator.setPos(ElevatorConstants.MAX_HEIGHT);
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < 50000) {
            set(-0.1);

            if (getAmp() > ArmConstants.AMP_THRESHOLD) {
                armOffset = armEncoder.get() - ArmConstants.MIN_ANGLE;
                SaveData.saveData("armOffset", armOffset);

                while (System.currentTimeMillis() - startTime < 50000) {
                    set(0.1);

                    if (getAmp() > ArmConstants.AMP_THRESHOLD) {
                        pulse2Degree = ArmConstants.MAX_ANGLE / (armEncoder.get() - armOffset);
                        SaveData.saveData("armPulse2Degree", pulse2Degree);

                        while (getAngle() != 0) {
                            if (System.currentTimeMillis() - startTime >= 50000) {
                                System.out.println("Calibración terminada por tiempo de espera.");
                                set(0);
                                return;
                            }

                            setPos(0);
                        }

                        System.out.println("Calibración terminada.");
                        set(0);
                        return;
                    }
                }

                System.out.println("Calibración terminada por tiempo de espera.");
                set(0);
                return;
            }
        }

        System.out.println("Calibración terminada por tiempo de espera.");
        set(0);
        RobotContainer.elevator.setPos(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Arm Current", getAmp());
    }
}
