package frc.robot.subsystems.outake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class OutakeIO {
    private final SparkMax motor;
    private boolean state;

    /**
     * Configures follower motors to mirror their respective leaders.
     */
    public OutakeIO() {
        motor = new SparkMax(OutakeConstants.MOTOR_ID, MotorType.kBrushless);
        state = false;
    }

    /**
     * @return The leader motor.
     */
    public SparkMax getMotor() {
        return motor;
    }

    /**
     * 
     */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        state = true;
    }

    /**
     * Stops both motors immediately.
     */
    public void stop() {
        motor.stopMotor();
        state = false;
    }

    public boolean getState() {
        return state;
    }
}
