package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class EndEffectorIO {
    private final TalonFX motor;

    /**
     * Configures follower motors to mirror their respective leaders.
     */
    public EndEffectorIO() {
        motor = new TalonFX(EndEffectorConstants.MOTOR_ID);
    }

    /**
     * @return The leader motor.
     */
    public TalonFX getMotor() {
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
        return motor.getStatorCurrent().getValueAsDouble();
    }
}
