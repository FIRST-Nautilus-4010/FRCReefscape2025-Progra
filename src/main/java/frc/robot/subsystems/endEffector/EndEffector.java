package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.endEffector.commands.Intake;
import frc.robot.subsystems.endEffector.commands.Outake;

public class EndEffector extends SubsystemBase{
    private final EndEffectorIO io;

    public EndEffector() {
        io = new EndEffectorIO();
    }

    public Command intake() {
        return new Intake(io, this).withName("OutakeIntakeCmd");
    }

    public Command outake() {
        return new InstantCommand(() -> io.set(.3));
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Outake state", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }
}
