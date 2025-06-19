package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    TalonFX krakenL = new TalonFX(16);
    TalonFX krakenR = new TalonFX(15);
    DutyCycleEncoder angleEnc = new DutyCycleEncoder(2);
    DutyCycleEncoder motorLEnc = new DutyCycleEncoder(3);
    PIDController anglePID = new PIDController(1, 0, 0);
    PIDController heightPID = new PIDController(1, 0, 0);
    boolean onMaxLimit = false;
    boolean onMinLimit = true;
    double angleOffset = 0;
    boolean runToPosition = false;
    double angleSetpoint = 0;
    double heightSetpoint = 0;

    public void setPower(double rotPower, double upPower, boolean runToPosition) {
        if (getHeight() <= 0.2 || (getAngle() > 270 && getAngle() < 90)) {
            rotPower = 0;
            upPower = rotPower;
        }

        double denom = Math.max(1, Math.abs(upPower + rotPower));
        krakenL.set((upPower - rotPower) / denom);
        krakenR.set((-upPower - rotPower) / denom);

        this.runToPosition = runToPosition;
    }

    public void setPower(double rotPower, double upPower) {
        if (getHeight() <= 0.2 || (getAngle() > 270 && getAngle() < 90)) {
            rotPower = 0;
            upPower = rotPower;
        }

        double denom = Math.max(1, Math.abs(upPower + rotPower));
        krakenL.set((upPower - rotPower) / denom);
        krakenR.set((-upPower - rotPower) / denom);

        this.runToPosition = false;
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

        angleSetpoint = angle;
        heightSetpoint = height;

        runToPosition = true;
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

    private final NetworkTableEntry armAngleEntry = SmartDashboard.getEntry("Arm Angle");
    private final NetworkTableEntry heightEntry = SmartDashboard.getEntry("Height");

    int loopCounter = 0;

    @Override
    public void periodic() {
        loopCounter++;

        if (loopCounter >= 10) { // cada 10 ciclos (~200ms)
            armAngleEntry.setDouble(getAngle());
            heightEntry.setDouble(getHeight());
            loopCounter = 0;
        }

        if (onMaxLimit && getAngle() >= 0 && getAngle() < 180) angleOffset += 360;
        if (onMinLimit && getAngle() <= 360 && getAngle() > 180) angleOffset -= 360;

        onMaxLimit = getAngle() > 350;
        onMinLimit = getAngle() < 10;

        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Height", getHeight());

        if (runToPosition) {
            if (a == 0) anglePoints = getInterPts(getTotalAngle(), angleSetpoint);
            if (h == 0) heightPoints = getInterPts(getHeight(), heightSetpoint);

            double desiredAngle = (a < anglePoints.size()) ? anglePoints.get(a) : angleSetpoint;
            double desiredHeight = (h < heightPoints.size()) ? heightPoints.get(h) : heightSetpoint;

            double anglePwr = 0, heightPwr = 0;

            if (Math.abs(desiredAngle - getTotalAngle()) < 0.1) a++;
            else anglePwr = anglePID.calculate(getTotalAngle(), desiredAngle);

            if (Math.abs(desiredHeight - getHeight()) < 0.005) h++;
            else heightPwr = heightPID.calculate(getHeight(), desiredHeight);

            setPower(anglePwr, heightPwr, true);

            if (a >= anglePoints.size() - 1 && h >= heightPoints.size() - 1) {
                runToPosition = false;
                a = 0;
                h = 0;
            }
        }
    }

}
