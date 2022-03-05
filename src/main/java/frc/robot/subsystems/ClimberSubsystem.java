// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum Stages {
  Extend, Grab, Pull, Climbing
}

public class ClimberSubsystem extends SubsystemBase {
  TalonFX m_winch = new TalonFX(Constants.c.k_climber);
  Servo m_ratchet = new Servo(Constants.c.k_servo);
  Solenoid m_piston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.c.k_piston);
  Solenoid m_grabber = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.c.k_claw);
  Stages stage;

  public ClimberSubsystem() {
    stage = Stages.Extend;
    Shuffleboard.getTab("Drive Tab").addString("Climb Stage", () -> stage.toString()).withSize(2, 1).withPosition(6, 2);
  }

  public void setWinchPower(double power) {
    if(power > 0 && m_ratchet.get() == 0.5) m_winch.set(ControlMode.PercentOutput, 0);
    else m_winch.set(ControlMode.PercentOutput, power);
  }

  public void setServo(double angle) {
    m_ratchet.set(angle);
  }

  public void ratchet(boolean on) {
    if(on) m_ratchet.set(0.5);
    else m_ratchet.set(0.25);
  }

  public void climb() {
    switch(stage) {
      case Extend:
        ratchet(false);
        m_piston.set(true);
        stage = Stages.Grab;
        break;
      case Grab:
        m_grabber.set(true);
        stage = Stages.Pull;
        break;
      case Pull:
        m_piston.set(false);
        stage = Stages.Climbing;
        break;
      case Climbing:
        ratchet(true);
        m_grabber.set(false);
        break;
    }
  }

  @Override
  public void periodic() {
    // SmartDashboard.putString("Climber Stage", stage.toString());
    // Check if final climbing and motor is stalled
    // if(stage == Stages.Climbing && m_winch.getMotorOutputPercent() > 0.1 && m_winch.getSelectedSensorVelocity() < 100) {
    //   ratchet(true);
    // }
  }
}
