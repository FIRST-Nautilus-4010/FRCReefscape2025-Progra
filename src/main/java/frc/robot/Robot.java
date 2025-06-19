// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Coprocessor;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private Coprocessor coprocessor;
  private boolean ranTest = false;

  @Override
  public void robotInit() {
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (!ranTest) {
      try {
        coprocessor = new Coprocessor();
        coprocessor.periodic();

        if (coprocessor.isConnected()) {
          System.out.println("Coprocesador conectado correctamente.");
          int[][] path = coprocessor.getPathfind(0, 0, 100, 100);

          for (int i = 0; i < path.length; i++) {
            System.out.println("Punto " + i + ": (" + path[i][0] + ", " + path[i][1] + ")");
          }

          coprocessor.closeConnection(); // Cierra la conexión al finalizar
          ranTest = true; // Solo se ejecuta una vez
        } else {
          System.err.println("No se pudo conectar al coprocesador.");
        }
      } catch (IOException e) {
        e.printStackTrace();
      }
  }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = RobotContainer.getAutonomouCmd();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    TeleOp.initialize();
  }


  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    Command testCommand = RobotContainer.getTestCommand();

    if (testCommand != null) {
      testCommand.schedule();
    }
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


}
