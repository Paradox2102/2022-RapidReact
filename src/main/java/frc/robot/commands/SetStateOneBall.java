// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.States;
import frc.robot.subsystems.ScottySubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetStateOneBall extends InstantCommand {
  ScottySubsystem m_scottySubsystem;
  public SetStateOneBall(ScottySubsystem scottySubsystem) {
    // addRequirements(m_scottySubsystem);
    m_scottySubsystem = scottySubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_scottySubsystem.setState(States.OneBall);
  }
}
