// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.subsystems.HoodSubsystem;

public class HoodCommand extends CommandBase {
  HoodSubsystem m_hoodSubystem;
  DoubleSupplier m_angle;
  public HoodCommand(HoodSubsystem hoodSubsystem, DoubleSupplier angle) {
    m_hoodSubystem = hoodSubsystem;
    m_angle = angle;
    addRequirements(hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Log("Hood Command", 1, "Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_hoodSubystem.setHoodAngle((m_angle.getAsDouble()+1)/2);
    // SmartDashboard.putNumber("Hood Value", (m_angle.getAsDouble()+1)/2);
    m_hoodSubystem.setHoodAngle(0.418);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.Log("Hood Command", 1, "End");
    m_hoodSubystem.setHoodAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
