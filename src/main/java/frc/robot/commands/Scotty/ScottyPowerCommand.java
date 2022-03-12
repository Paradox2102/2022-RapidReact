// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scotty;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.States;
import frc.robot.subsystems.ScottySubsystem;

public class ScottyPowerCommand extends CommandBase {
  ScottySubsystem m_scottySubsystem;
  double m_power;
  double m_time = 0;
  double m_timer = 0;
  public ScottyPowerCommand(ScottySubsystem scottySubsystem, double power) {
    m_scottySubsystem = scottySubsystem;
    m_power = power;
    addRequirements(scottySubsystem);
  }
  public ScottyPowerCommand(ScottySubsystem scottySubsystem, double power, double time) {
    m_scottySubsystem = scottySubsystem;
    m_power = power;
    m_time = time;
    addRequirements(scottySubsystem);
  }

  @Override
  public void initialize() {
    m_scottySubsystem.runScotty(m_power);
    m_timer = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_scottySubsystem.runScotty(0);
    m_scottySubsystem.setState(States.None);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_time != 0 ? System.currentTimeMillis() - m_timer >= m_time : false;
  }
}
