// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.management.timer.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.States;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScottySubsystem;

public class IntakeCommand extends CommandBase {
  IntakeSubsystem m_intakeSubsystem;
  ScottySubsystem m_scottySubsystem;
  double m_power;
  double m_time;
  double m_timer = 0;
  boolean m_timing = false;
  public IntakeCommand(IntakeSubsystem intakeSubsystem, ScottySubsystem scottySubsystem, double power) {
    m_intakeSubsystem = intakeSubsystem;
    m_scottySubsystem = scottySubsystem;
    m_power = power;
    addRequirements(intakeSubsystem);
  }
  public IntakeCommand(IntakeSubsystem intakeSubsystem, ScottySubsystem scottySubsystem, double power, double time) {
    m_intakeSubsystem = intakeSubsystem;
    m_scottySubsystem = scottySubsystem;
    m_power = power;
    m_time = time;
    m_timing = false;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    Logger.Log("Intake Command", 1, "Initialized");
    // m_intakeSubsystem.deployIntake(true);
    m_intakeSubsystem.runIntake(m_power);
    if(m_timing) m_timer = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    if(m_scottySubsystem.getState() == States.Full && m_power > 0) m_intakeSubsystem.runIntake(0);
    else m_intakeSubsystem.runIntake(m_power);
  }
  @Override
  public void end(boolean interrupted) {
    // m_intakeSubsystem.deployIntake(false);
    m_intakeSubsystem.runIntake(0);
  }

  @Override
  public boolean isFinished() {
    if(m_timing) return System.currentTimeMillis() - m_timer >= m_time;
    else return false;
  }
}
