package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.controls.Follower;
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
        return leader.getPosition().getValue().magnitude() / ShoulderConstants.ROT_2_RADIAN;
    }

    public double getHeight() {
        return -Math.cos(getAngle()) * ShoulderConstants.UPPER_ARM_LENGTH;
    }

    public double getVelocity() {
        return leader.getVelocity().getValue().magnitude() / ShoulderConstants.ROT_2_RADIAN;
    }

    public TalonFX getLeaderMotor() {
        return leader;
    }

    public void stop() {
        leader.stopMotor();
    }
}