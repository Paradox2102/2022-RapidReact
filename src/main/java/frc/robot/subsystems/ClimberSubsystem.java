// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
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
    m_climbFollower.follow(m_climb);
    // Shuffleboard.getTab("Drive Tab").addString("Climb Stage", () -> stage.toString()).withSize(2, 1).withPosition(6, 2);
  }
  public void setClimbPower(double power) {
    if(DriverStation.getMatchTime() > 30) {
      System.out.println("BAD!!! CANNOT CLIMB YET");
      return;
    }
    m_climb.set(ControlMode.PercentOutput, power);
  }

  public void toggleRotate() {
    if(DriverStation.getMatchTime() > 30) {
      System.out.println("BAD!!! CANNOT CLIMB YET");
      return;
    }
    rotated = !rotated;
    m_rotater.set(rotated);
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
    // SmartDashboard.putString("Climber Stage", stage.toString());
    // Check if final climbing and motor is stalled
    // if(stage == Stages.Climbing && m_winch.getMotorOutputPercent() > 0.1 && m_winch.Velocity() < 100) {
    //   ratchet(true);
    // }
  }
}
