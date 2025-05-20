package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    TalonFX krakenL = new TalonFX(0);
    TalonFX krakenR = new TalonFX(0);
    DutyCycleEncoder angleEnc = new DutyCycleEncoder(3);
    DutyCycleEncoder motorLEnc = new DutyCycleEncoder(3);
    PIDController anglePID = new PIDController(1, 0, 0);
    PIDController heightPID = new PIDController(1, 0, 0);
    boolean onMaxLimit = false;
    boolean onMinLimit = true;
    double angleOffset = 0;
    boolean runToPosition = false;
    double angleSetpoint = 0;
    double heightSetpoint = 0;

    public void setPower(double rotPower, double upPower) {
        if (getHeight() <= 0.2 || (getAngle() > 270 && getAngle() < 90)) {
            rotPower = 0;
            upPower = rotPower;
        }

        double denom = Math.max(1, Math.abs(upPower + rotPower));
        krakenL.set((upPower - rotPower) / denom);
        krakenR.set((-upPower - rotPower) / denom);
    }

    public double getAngle() {
        return angleEnc.get() / 0.5;
    }

    public double getTotalAngle() {
        return angleOffset + getAngle();
    }

    public double getHeight() {
        return motorLEnc.get() - getTotalAngle() / .5;
    }

    public void setPos(double angle, double height) {
        if (Math.abs(angle - getAngle()) > 180){
            angle -= 360;
        }
        setPower(anglePID.calculate(getTotalAngle(), angle), heightPID.calculate(getHeight(), height));
    }

    public ArrayList<Double> getInterPts(double measurment, double setpoint) {
        double P0 = measurment;
        double P3 = setpoint;
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

        points.add(P3);

        return points;
    }

    int a = 0;
    int h = 0;
    ArrayList<Double> anglePoints;
    ArrayList<Double> heightPoints;

    @Override
    public void periodic() {
        if (onMaxLimit && getAngle() >= 0 && getAngle() < 180) {
            angleOffset += 360;
        }

        if (onMinLimit && getAngle() <= 360 && getAngle() > 180) {
            angleOffset -= 360;
        }

        onMaxLimit = getAngle() > 350;
        onMinLimit = getAngle() < 10;

        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Height", getHeight());

        if (runToPosition) {
            if (a == 0) {
                anglePoints = getInterPts(getTotalAngle(), angleSetpoint);
            }

            if (h == 0) {
                heightPoints = getInterPts(getHeight(), heightSetpoint);
            }
            
            double desiredAngle = anglePoints.get(a);

            double anglePwr = 0;
            if (Math.abs(desiredAngle - getTotalAngle()) < .1) {
                a++;
            } else {
                anglePwr = anglePID.calculate(getTotalAngle(), desiredAngle);
            }


            double desiredHeight = heightPoints.get(h);

            double heightPwr = 0;
            if (Math.abs(desiredHeight - getHeight()) < .005) {
                h++;
            } else {
                heightPwr = heightPID.calculate(getHeight(), desiredHeight);
            }

            setPower(anglePwr, heightPwr);

            if (a >= anglePoints.size() - 1) {
                a = 0;
            }

            if (h >= heightPoints.size() - 1) {
                h = 0;
            }
        }
    }    
}
