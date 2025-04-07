package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.ArrayList;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.HardwareMap;

public class Arm extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(HardwareMap.ARM, MotorType.kBrushless);
    private final SparkMax armMotor1 = new SparkMax(HardwareMap.ARM1, MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(HardwareMap.ARM_ENC);
    private final PIDController armPID = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);

    private double angle = 0;
    private boolean runToPosition = false;

    public Arm(boolean armInverted) {
        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.inverted(armInverted);
        armMotor.configure(armMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);

        SparkMaxConfig armMotor1Config = new SparkMaxConfig();
        armMotor1Config.follow(armMotor.getDeviceId());
        armMotor1.configure(armMotor1Config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
    }

    public double getAngle() {
        return (armEncoder.get() - ArmConstants.OFFSET) * ArmConstants.PULSE2DEGREE;
    }

    public void set(double speed) {
        if ((getAngle() >= ArmConstants.MAX_ANGLE - 1 || getAmp() < ArmConstants.AMP_THRESHOLD) && speed > 0 ) {
            stop();
        } else if ((getAngle() <= ArmConstants.MIN_ANGLE + 1 || getAmp() < ArmConstants.AMP_THRESHOLD) && speed < 0) {
            stop();
        } else {
            armMotor.set(speed);
        }
    }

    public void setPos(double angle) {
        this.angle = angle;
    }

    public ArrayList<Double> getInterPts() {
        double P0 = getAngle();
        double P3 = angle;
        double n = Math.round(Math.abs(P3 - P0) / 2);
        double k = 0.0186430923726995846 * n;
        P0 *= k;
        P3 *= k;
        
        ArrayList<Double> points = new ArrayList<>();
        for (int i = 0; i <= n; i++) {
            double t = i / n;
            double point = P0 + (P3 - P0) / (1 + Math.pow(Math.E, -8 * (t- 0.5)));

            points.add(point);
        }


        return points;
    }

    public void setRunToPosition(boolean runToPosition) {
        this.runToPosition = runToPosition;
    }

    public double getAmp() {
        return armMotor.getOutputCurrent();
    }

    public void stop() {
        armMotor.stopMotor();
    }

    int i = 0;
    ArrayList<Double> points;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Arm Current", getAmp());

        if (runToPosition) {
            if (i == 0) {
                points = getInterPts();
            }
            
            double desiredPos = points.get(i);
            if (Math.abs(desiredPos - getAngle()) < .1) {
                i++;
            } else {
                armMotor.set(armPID.calculate(getAngle(), desiredPos));
            }

            if (i >= points.size() - 1) {
                i = 0;
            }
        } 
    }
}