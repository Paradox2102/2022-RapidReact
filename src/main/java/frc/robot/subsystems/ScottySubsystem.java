// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum States {
  None, MoveOne, OneBall, MoveTwo, Full
}

public class ScottySubsystem extends SubsystemBase {
  TalonFX m_scotty = new TalonFX(Constants.k_scotty);
  DigitalInput m_scotClose = new DigitalInput(Constants.k_scotClose);
  DigitalInput m_scotMid = new DigitalInput(Constants.k_scotMid);
  DigitalInput m_scotFar = new DigitalInput(Constants.k_scotFar);

  States state;
  boolean run;
  double power;

  public ScottySubsystem() {
    state = States.None;
    run = false;
    // this.power = power;
  }

  public void runScotty(double power) {
    m_scotty.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    boolean bot = m_scotFar.get();
    boolean mid = m_scotFar.get();
    boolean top = m_scotFar.get();

    switch(state) {
      case None:
        if(bot) {
          run = true;
          state = States.MoveOne;
        }
        break;
      case MoveOne:
        if(mid) {
          run = false;
          state = States.OneBall;
        }
        break;
      case OneBall:
        if(bot) {
          run = true;
          state = States.MoveTwo;
        }
        break;
      case MoveTwo:
        if(top) {
          run = false;
          state = States.Full;
        }
        break;
      case Full:
        if(bot) {
          // Reverse intake
        }
        break;
    }
    if(top) run = false;
    if(run) runScotty(power);
  }
}
