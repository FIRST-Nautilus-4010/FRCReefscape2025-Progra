package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Claw extends SubsystemBase{
    double position = 0;

    final CANSparkMax rMotor = new CANSparkMax(Constants.HardwareMap.CLAW_R_MOTOR, MotorType.kBrushless);
    final CANSparkMax lMotor = new CANSparkMax(Constants.HardwareMap.CLAW_L_MOTOR, MotorType.kBrushless);
    final CANSparkMax clawMotor = new CANSparkMax(Constants.HardwareMap.CLAW_MOTOR, MotorType.kBrushless);
    final RelativeEncoder clawEncoder = clawMotor.getEncoder();

    final PIDController clawPID = new PIDController(Constants.ClawConstants.P, Constants.ClawConstants.I, Constants.ClawConstants.D);

    public void setClawMotorPos(double position) {
        this.position = position;
    }

    public void setRMotor(double power) {
        rMotor.set(power);

    }

    public void setLMotor(double power) {
        lMotor.set(power);
        
    }

    public void setMotors(double power) {
        setRMotor(power);
        setLMotor(power);
    }

    public double getClawMotorPos() {
        return clawEncoder.getPosition();
    }

    @Override
    public void periodic() {
        clawMotor.set(clawPID.calculate(clawEncoder.getPosition(), position));
        SmartDashboard.putNumber("Claw pos", getClawMotorPos());
    }

}

// ╱|、
//(˚ˎ 。7  
// |、˜〵          
// じしˍ,)ノ    