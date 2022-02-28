// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  TalonFX m_shooter = new TalonFX(Constants.c.k_shooter);
  TalonFX m_shooterFollower = new TalonFX(Constants.c.k_shooterFollower);

  double k_f = 0.061;
  double k_p = 0.5;
  double k_i = 0.001;
  double k_iZone = 100;
  int k_timeout = 30;

  public ShooterSubsystem() {
    m_shooter.configFactoryDefault();
    m_shooterFollower.configFactoryDefault();
    m_shooter.setInverted(false);
    m_shooterFollower.setInverted(true);
    m_shooterFollower.follow(m_shooter);
    m_shooter.setNeutralMode(NeutralMode.Coast);
    m_shooterFollower.setNeutralMode(NeutralMode.Coast);

    m_shooter.config_kF(0, k_f, k_timeout); 
    m_shooter.config_kP(0, k_p, k_timeout);
    m_shooter.config_kI(0, k_i, k_timeout);
    m_shooter.config_IntegralZone(0, k_iZone, k_timeout);
  }

  public void setShooterPower(double power) {
    m_shooter.set(ControlMode.PercentOutput, power);
  }

  public void setShooterSpeed(double speed) {
    m_shooter.set(ControlMode.Velocity, speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterSpeed", m_shooter.getSelectedSensorVelocity());
  }
}
