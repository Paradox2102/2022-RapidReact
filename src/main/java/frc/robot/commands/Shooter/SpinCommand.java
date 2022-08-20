// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinCommand extends CommandBase {

  ShooterSubsystem m_shooterSubsystem; 
  double m_power;
  boolean m_shootLow;

  public SpinCommand(ShooterSubsystem shooterSubsytem, double power, boolean shootLow) {
    m_shooterSubsystem = shooterSubsytem;
    m_power = power;
    m_shootLow = shootLow;

    addRequirements(m_shooterSubsystem);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Log("Spin Up Command", 1, "Initialized");
    m_shooterSubsystem.setLow(m_shootLow);
    m_shooterSubsystem.setShooterSpeed(m_power);
    m_shooterSubsystem.setBackWheelPower(m_shootLow ? 0.5 : -0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_shooterSubsystem.setShooterPower((m_power + 1) / 2); //.getAsDouble() 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Spin Up Command End");
    m_shooterSubsystem.setShooterPower(0);
    m_shooterSubsystem.setBackWheelPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
