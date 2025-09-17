package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class EndEffectorIO {
    private final SparkMax motor;

    /**
     * Configures follower motors to mirror their respective leaders.
     */
    public EndEffectorIO() {
        motor = new SparkMax(EndEffectorConstants.MOTOR_ID, MotorType.kBrushless);
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
    public void set(double power) {
        motor.set(power);
    }

    /**
     * Stops both motors immediately.
     */
    public void stop() {
        motor.stopMotor();
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }
}
