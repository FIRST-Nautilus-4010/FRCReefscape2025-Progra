package frc.robot.subsystems.outake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OutakeSubsystem extends SubsystemBase{
    private final OutakeIO io;

    public OutakeSubsystem() {
        io = new OutakeIO();
    }

    public Command intake() {
        return new InstantCommand(() -> io.setVoltage(10), this).withName("OutakeIntakeCmd");
    }

    public Command outake() {
        return new InstantCommand(() -> io.setVoltage(-10), this).withName("OutakeOutakeCmd");
    }

    public Command stop() {
        return new InstantCommand(() -> io.stop(), this).withName("OutakeStopCmd");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean(getName(), io.getState());
    }
}
