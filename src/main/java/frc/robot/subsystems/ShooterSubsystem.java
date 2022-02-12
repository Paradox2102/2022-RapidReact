// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  TalonFX m_shooter = new TalonFX(Constants.k_shooter);
  TalonFX m_shooterFollower = new TalonFX(Constants.k_shooterFollower);

  public ShooterSubsystem() {
    m_shooter.setInverted(true);
    m_shooterFollower.follow(m_shooter);
  }

  public void setShooterPower(double power) {
    m_shooter.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterSpeed", m_shooter.getMotorOutputPercent());
  }
}
