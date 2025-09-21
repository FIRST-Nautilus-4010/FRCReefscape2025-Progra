package frc.robot.subsystems.shoulder;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShoulderController {
    private final TalonFX leader;
    private final TalonFXConfiguration configuration;
    private final MotionMagicExpoVoltage posRequest;

    private final ShoulderIO io;
    private static Supplier<Double> robotAcc = () -> 0.0;

    public ShoulderController(ShoulderIO io) {
        this.leader = io.getLeaderMotor();
        this.io = io;

        posRequest = new MotionMagicExpoVoltage(0).withSlot(0);

        configuration = new TalonFXConfiguration();

        setPositionSlotGains();
        setMMSettings();

        leader.getConfigurator().apply(configuration);
    }

    public static void setRobotAccSupplier(Supplier<Double> supplier) {
        robotAcc = supplier;
    }

    private void setPositionSlotGains() {
        var slot0 = configuration.Slot0;
        slot0.kG = ShoulderConstants.POS_KG;
        slot0.kS = ShoulderConstants.POS_KS;
        slot0.kV = ShoulderConstants.POS_KV;
        slot0.kA = ShoulderConstants.POS_KA;
        slot0.kP = ShoulderConstants.POS_KP;
        slot0.kI = ShoulderConstants.POS_KI;
        slot0.kD = ShoulderConstants.POS_KD;
    }

    private void setMMSettings() {
        var motionMagic = configuration.MotionMagic;
        motionMagic.MotionMagicCruiseVelocity = ShoulderConstants.MAGIC_MOTION_VELOCITY;
        motionMagic.MotionMagicAcceleration = ShoulderConstants.MAGIC_MOTION_ACCELERATION;
        motionMagic.MotionMagicJerk = ShoulderConstants.MAGIC_MOTION_JERK;
        motionMagic.MotionMagicExpo_kV = ShoulderConstants.MAGIC_MOTION_EXPO_KV; 
        motionMagic.MotionMagicExpo_kA = ShoulderConstants.MAGIC_MOTION_EXPO_KA;
    }

    public void update() {
        var slot0 = configuration.Slot0;
        slot0.kG = ShoulderConstants.POS_KG * Math.sin(io.getAngle());
        slot0.kA = ShoulderConstants.POS_KA * robotAcc.get() * -Math.cos(io.getAngle());

        leader.getConfigurator().apply(configuration);
    }

    public void rotateTo(double angle) {
    leader.setControl(posRequest.withPosition(angle / ShoulderConstants.ROT_2_RADIAN));
    }
}