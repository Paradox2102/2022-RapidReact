// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.subsystems.ClimberSubsystem;

public class ServoThrottleCommand extends CommandBase {
  ClimberSubsystem m_climberSubsystem;
  DoubleSupplier m_angle;
  public ServoThrottleCommand(ClimberSubsystem climberSubsystem, DoubleSupplier angle) {
    m_climberSubsystem = climberSubsystem;
    m_angle = angle;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Log("Servo Command", 1, "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = (m_angle.getAsDouble() + 1)/2;
    m_climberSubsystem.setServo(angle);
    SmartDashboard.putNumber("Ratchet Servo Angle", angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.Log("Servo Command", 1, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
