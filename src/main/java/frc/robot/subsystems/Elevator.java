package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.ArrayList;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ElevatorConstants;
import frc.robot.utils.Constants.HardwareMap;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor = new SparkMax(HardwareMap.ELEVATOR, MotorType.kBrushless);
    private final SparkMax elevatorMotor1 = new SparkMax(HardwareMap.ELEVATOR1, MotorType.kBrushless);
    private final DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(HardwareMap.ELEVATOR_ENC);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(HardwareMap.BOTTOM_LIMIT);
    private final DigitalInput topLimitSwitch = new DigitalInput(HardwareMap.TOP_LIMIT);
    private final PIDController elevatorPID = new PIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D);

    private double position = 0;
    private boolean runToPosition = false;

    public Elevator(boolean isInverted) {
        SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
        elevatorMotorConfig.inverted(isInverted);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig elevatorMotor1Config = new SparkMaxConfig();
        elevatorMotor1Config.follow(elevatorMotor.getDeviceId());
        elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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

    public double getPos() {
        return (elevatorEncoder.get() - ElevatorConstants.OFFSET) * ElevatorConstants.PULSE2M;
    }

    public void set(double speed) {
        if (speed > 0 && (getTopLimitSwitch() || getPos() > ElevatorConstants.MAX_HEIGHT || getAmp() < ElevatorConstants.AMP_THRESHOLD)) {
            stop();
        } else if (speed < 0 && (getBottomLimitSwitch() || getPos() < 0 || getAmp() < ElevatorConstants.AMP_THRESHOLD)) {
            stop();
        } else {
            elevatorMotor.set(speed);
        }
        
    }

    public void setPos(double position) {
        this.position = position;
    }

    public void moveToPosition() {
        double P0 = getPos();
        double P3 = position;
        double k = 0.0186430923726995846 * (Math.abs(P3 - P0));
        P0 *= k;
        P3 *= k;
        
        ArrayList<Double> points = new ArrayList<>();
        for (int i = 0; i <= 200; i++) {
            float t = i / 200;
            double point = P0 + (P3 - P0) / (1 + Math.pow(Math.E, -8 * (t- 0.5)));

            points.add(point);
        }

        for (int i = 0; i < points.size(); i++) {
            double desiredPos = points.get(i);
            while (Math.abs(desiredPos - getPos()) < .1) {
                set(elevatorPID.calculate(getPos(), desiredPos));
            }
        }
    }

    public void setRunToPosition(boolean runToPosition) {
        this.runToPosition = runToPosition;
    }

    public void stop() {
        elevatorMotor.stopMotor();
    }

    public void update() {
        SmartDashboard.putNumber("Elevator Position", getPos());
        SmartDashboard.putNumber("Elevator amp", getAmp());

        if (runToPosition) {
            moveToPosition();
        } 
    }

}