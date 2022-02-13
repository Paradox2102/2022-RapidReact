// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  IntakeSubsystem m_intakeSubsystem;
  double m_power;
  public IntakeCommand(IntakeSubsystem intakeSubsystem, double power) {
    m_intakeSubsystem = intakeSubsystem;
    m_power = power;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    Logger.Log("Intake Command", 1, "Initialized");
    // m_intakeSubsystem.deployIntake(true);
    m_intakeSubsystem.runIntake(m_power);
  }

  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {
    // m_intakeSubsystem.deployIntake(false);
    m_intakeSubsystem.runIntake(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
