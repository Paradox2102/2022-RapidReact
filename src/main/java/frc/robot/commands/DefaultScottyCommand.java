// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.States;
import frc.robot.subsystems.ScottySubsystem;

public class DefaultScottyCommand extends CommandBase {
  ScottySubsystem m_scottySubsystem;
  double m_power;
  boolean m_run = false;
  public DefaultScottyCommand(ScottySubsystem scottySubsystem, double power) {
    m_scottySubsystem = scottySubsystem;
    m_power = power;
    addRequirements(scottySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Log("DefaultScotty", 1, "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean bot = m_scottySubsystem.getBotSensor();
    boolean mid = m_scottySubsystem.getMidSensor();
    boolean top = m_scottySubsystem.getTopSensor();

    States state = m_scottySubsystem.getState();

    switch(state) {
      case None:
        if(bot) {
          m_run = true;
          m_scottySubsystem.setState(States.MoveOne);
        }
        break;
      case MoveOne:
        if(mid) {
          m_run = false;
          m_scottySubsystem.setState(States.OneBall);
        }
        break;
      case OneBall:
        if(bot) {
          m_run = true;
          m_scottySubsystem.setState(States.MoveTwo);
        }
        break;
      case MoveTwo:
        if(top) {
          m_run = false;
          m_scottySubsystem.setState(States.Full);
        }
        break;
      case Full:
        break;
    }
    if(top) m_run = false;
    if(m_run) m_scottySubsystem.runScotty(m_power);
    else m_scottySubsystem.runScotty(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.Log("DefaultScotty", 1, "End");
    m_run = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
