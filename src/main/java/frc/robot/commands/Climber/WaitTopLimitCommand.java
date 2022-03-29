// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class WaitTopLimitCommand extends CommandBase {
  ClimberSubsystem m_climberSubsystem;
  public WaitTopLimitCommand(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;
  }

  @Override
  public boolean isFinished() {
    return m_climberSubsystem.getBothTop();
  }
}
