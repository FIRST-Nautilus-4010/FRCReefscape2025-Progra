package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShoulderIO {
    private final TalonFX leader;
    private final TalonFX follower;

    public ShoulderIO() {
        leader = new TalonFX(ShoulderConstants.KRAKEN_L_ID);
        follower = new TalonFX(ShoulderConstants.KRAKEN_R_ID);

        follower.setControl(new Follower(leader.getDeviceID(), false));
    }

    public double getAngle() {
        return leader.getPosition().getValue().magnitude();
    }

    public void setVoltage(double voltage) {
        leader.setControl(new VoltageOut(voltage));
    }

    public TalonFX getLeaderMotor() {
        return leader;
    }

    public void stop() {
        leader.stopMotor();
    }
}