// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.commands.IntakeCommand;

public class ScottySubsystem extends SubsystemBase {
  TalonFX m_scotty = new TalonFX(Constants.c.k_scotty);
  DigitalInput m_scotClose = new DigitalInput(Constants.c.k_scotClose);
  DigitalInput m_scotMid = new DigitalInput(Constants.c.k_scotMid);
  DigitalInput m_scotFar = new DigitalInput(Constants.c.k_scotFar);

  States m_state = States.None;
  boolean m_run;

  public ScottySubsystem() {
    m_scotty.setInverted(false);
    m_run = false;
  }

  public void runScotty(double power) {
    m_scotty.set(ControlMode.PercentOutput, power);
  }

  public void setState(States state) {
    m_state = state;
  }
  public States getState() {
    return m_state;
  }
  public boolean getBotSensor() {
    return !m_scotClose.get();
  }
  public boolean getMidSensor() {
    return !m_scotMid.get();
  }
  public boolean getTopSensor() {
    return !m_scotFar.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Top Sensor", getTopSensor());
    SmartDashboard.putBoolean("Mid Sensor", getMidSensor());
    SmartDashboard.putBoolean("Bot Sensor", getBotSensor());
    SmartDashboard.putString("Current State", m_state.toString());
  }
}
