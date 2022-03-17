// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.annotation.Nulls;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  DigitalInput m_magnetLeft = new DigitalInput(Constants.c.k_magneticLeft);
  DigitalInput m_magnetRight = new DigitalInput(Constants.c.k_magneticRight);

  DigitalInput m_switchLeft = new DigitalInput(Constants.c.k_switchLeft);
  DigitalInput m_switchRight = new DigitalInput(Constants.c.k_switchRight);

  TalonFX m_climb = new TalonFX(Constants.c.k_climber);
  TalonFX m_climbFollower = new TalonFX(Constants.c.k_climberFollower);
  
  Solenoid m_rotater = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.c.k_rotaterPiston);
  Solenoid m_break = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.c.k_breakerPiston);
  private boolean rotated;
  private boolean breaked;

  public ClimberSubsystem() {
    rotated = false;
    breaked = false;
    m_rotater.set(rotated);
    m_break.set(breaked);
    m_climb.setInverted(false);
    m_climbFollower.setInverted(true);
    m_climb.setNeutralMode(NeutralMode.Brake);
    m_climbFollower.setNeutralMode(NeutralMode.Brake);
    // m_climbFollower.follow(m_climb);
    // Shuffleboard.getTab("Drive Tab").addString("Climb Stage", () -> stage.toString()).withSize(2, 1).withPosition(6, 2);
  }
  public void setClimbPower(double power) {
    if(DriverStation.getMatchTime() > 30) {
      System.out.println("BAD!!! CANNOT CLIMB YET");
      return;
    }

    boolean leftMag = !m_magnetLeft.get();
    boolean rightMag = !m_magnetRight.get();
    boolean leftSw = !m_switchLeft.get();
    boolean rightSw = !m_switchRight.get();

    if(power < 0) {
      setBrake(true);
      m_climb.set(ControlMode.PercentOutput, !leftMag ? power : 0);
      m_climbFollower.set(ControlMode.PercentOutput, !rightMag ? power : 0);
    } else if(power > 0) {
      setBrake(true);
      m_climb.set(ControlMode.PercentOutput, !leftSw ? power : 0);
      m_climbFollower.set(ControlMode.PercentOutput, !rightSw ? power : 0);
    } else {
      setBrake(false);
      m_climb.set(ControlMode.PercentOutput, 0);
      m_climbFollower.set(ControlMode.PercentOutput, 0);
    }
  }

  public void toggleRotate() {
    if(DriverStation.getMatchTime() > 30) {
      System.out.println("BAD!!! CANNOT CLIMB YET");
      return;
    }
    rotated = !rotated;
    m_rotater.set(rotated);
  }

  public void setBrake(boolean on) {
    m_break.set(on);
    breaked = on;
  }

  public void toggleBreak() {
    if(DriverStation.getMatchTime() > 30) {
      System.out.println("BAD!!! CANNOT CLIMB YET");
      return;
    }
    breaked = !breaked;
    m_break.set(breaked);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Switch", !m_switchLeft.get());
    SmartDashboard.putBoolean("Right Switch", !m_switchRight.get());
    SmartDashboard.putBoolean("Left Magnet", !m_magnetLeft.get());
    SmartDashboard.putBoolean("Right Magnet", !m_magnetRight.get());
    // SmartDashboard.putString("Climber Stage", stage.toString());
    // Check if final climbing and motor is stalled
    // if(stage == Stages.Climbing && m_winch.getMotorOutputPercent() > 0.1 && m_winch.Velocity() < 100) {
    //   ratchet(true);
    // }
  }
}
