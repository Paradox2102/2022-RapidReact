// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum Stages {
  Extend, Grab, Swing
}

public class ClimberSubsystem extends SubsystemBase {
  TalonFX m_winch = new TalonFX(Constants.k_climber);
  Servo m_ratchet = new Servo(Constants.k_servo);
  Solenoid m_piston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.k_piston);
  Solenoid m_grabber = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.k_claw);
  Stages stage;

  public ClimberSubsystem() {
    stage = Stages.Extend;
  }

  public void ratchet() {
    // m_ratchet.set(0);
  }

  public void climb() {
    switch(stage) {
      case Extend:
        m_piston.set(true);
        // m_winch.set(ControlMode.PercentOutput, value); Release at same time as piston goes up
        stage = Stages.Grab;
        break;
      case Grab:
        m_grabber.set(true);
        m_piston.set(false);
        stage = Stages.Swing;
        break;
      case Swing:
        ratchet();
        // m_winch.set(ControlMode.PercentOutput, value); Release more
        m_grabber.set(false);
        break;
    }
  }

  @Override
  public void periodic() { }
}
