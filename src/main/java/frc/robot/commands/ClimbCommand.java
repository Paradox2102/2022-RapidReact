// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends CommandBase {
  ClimberSubsystem m_climberSubsystem;
  boolean end = false;
  public ClimbCommand(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    m_climberSubsystem.climb();
    end = true;
  }

  @Override
  public void execute() { }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return end;
  }
}
