package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIO {
    private final TalonFX krakenLBottom = new TalonFX(16);
    private final TalonFX krakenLTop = new TalonFX(15); 
    private final TalonFX krakenRBottom = new TalonFX(17);
    private final TalonFX krakenRTop = new TalonFX(18);

    public ElevatorIO() {
        krakenLTop.setControl(new  Follower(krakenLBottom.getDeviceID(), false));
        krakenRTop.setControl(new  Follower(krakenRBottom.getDeviceID(), false));
    }

    public TalonFX getLeftMotor() {
        return krakenLBottom;
    }

    public TalonFX getRightMotor() {
        return krakenRBottom;
    }

    public void setMotorPowers(double left, double right) {
        krakenLBottom.set(left);
        krakenRBottom.set(right);
    }
}

