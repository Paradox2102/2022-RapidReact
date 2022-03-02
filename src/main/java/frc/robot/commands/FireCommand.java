// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ScottySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FireCommand extends CommandBase {
  ScottySubsystem m_scottySubsystem;
  ShooterSubsystem m_shooterSubsystem;
  double m_power;
  final double k_deadZone = 150;
  public FireCommand(ScottySubsystem scottySubsystem, ShooterSubsystem shooterSubsystem, double scottyPower) {
    m_scottySubsystem = scottySubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_power = scottyPower;
    addRequirements(scottySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shooterSubsystem.getShooterSpeed() > m_shooterSubsystem.getShooterSetpoint() - k_deadZone) {
      m_scottySubsystem.runScotty(m_power);
    } else m_scottySubsystem.runScotty(0);
  }

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
