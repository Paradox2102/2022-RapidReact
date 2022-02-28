// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum Stages {
  Extend, Grab, Swing, Climbing, Climbed
}

public class ClimberSubsystem extends SubsystemBase {
  TalonFX m_winch = new TalonFX(Constants.c.k_climber);
  Servo m_ratchet = new Servo(Constants.c.k_servo);
  Solenoid m_piston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.c.k_piston);
  Solenoid m_grabber = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.c.k_claw);
  Stages stage;

  public ClimberSubsystem() {
    stage = Stages.Extend;
  }

  public void ratchet(boolean on) {
    if(on) m_ratchet.set(0);
    else m_ratchet.setAngle(45);
  }

  public void climb() {
    switch(stage) {
      case Extend:
        m_piston.set(true);
        // m_winch.set(ControlMode.PercentOutput, value); // Release at same time as piston goes up
        stage = Stages.Grab;
        break;
      case Grab:
        m_grabber.set(true);
        m_piston.set(false);
        stage = Stages.Swing;
        break;
      case Swing:
        // m_winch.set(ControlMode.PercentOutput, value); // Release more
        stage = Stages.Climbing;
        break;
      case Climbing:
        m_grabber.set(false);
        // m_winch.set(ControlMode.PercentOutput, -value);
        break;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Climber Stage", stage.toString());

    if(stage == Stages.Climbing /* && isStalled */) {
      ratchet(true);
      stage = Stages.Climbed;
    }
  }
}
