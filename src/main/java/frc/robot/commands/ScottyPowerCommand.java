// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ScottySubsystem;

public class ScottyPowerCommand extends CommandBase {
  ScottySubsystem m_scottySubsystem;
  double m_power;
  public ScottyPowerCommand(ScottySubsystem scottySubsystem, double power) {
    m_scottySubsystem = scottySubsystem;
    m_power = power;
    addRequirements(scottySubsystem);
  }

  @Override
  public void initialize() {
    m_scottySubsystem.runScotty(m_power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_scottySubsystem.runScotty(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
