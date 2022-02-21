// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinCommand extends CommandBase {

  ShooterSubsystem m_shooterSubsystem;
  DoubleSupplier m_power;

  public SpinCommand(ShooterSubsystem shooterSubsytem, DoubleSupplier power) {
    m_shooterSubsystem = shooterSubsytem;
    m_power = power;

    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Log("Spin Up Command", 1, "Initialized");
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setShooterPower((m_power.getAsDouble() + 1) / 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Spin Up Command End");
    m_shooterSubsystem.setShooterPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
