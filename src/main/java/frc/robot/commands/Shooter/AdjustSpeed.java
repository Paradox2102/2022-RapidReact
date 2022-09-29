// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AdjustSpeed extends CommandBase {
  ShooterSubsystem m_shooterSubsystem;
  DoubleSupplier m_speed;

  DoubleSupplier m_throttle; 
  double k_adjustmentRange = 500; 

  public AdjustSpeed(ShooterSubsystem shooterSubsystem, DoubleSupplier speed) {
    m_shooterSubsystem = shooterSubsystem;
    m_speed = speed;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setAdjustment(m_throttle.getAsDouble() * k_adjustmentRange);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setShooterPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}