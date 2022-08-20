// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  TalonFX m_shooter = new TalonFX(Constants.c.k_shooter); 
  TalonFX m_shooterFollower = new TalonFX(Constants.c.k_shooterFollower);
  TalonSRX m_backWheel = new TalonSRX(Constants.c.k_backWheel);

  double k_f = 0.061;
  double k_p = 0.25;
  double k_i = 0.001;
  double k_iZone = 100;
  int k_timeout = 30;

  double m_shooterSetpoint = 0;
  SimpleWidget m_amplifier;

  boolean m_isLow;

  double m_adjustment; 

  public ShooterSubsystem() {
    m_isLow = false;
    m_shooter.configFactoryDefault();
    m_shooterFollower.configFactoryDefault();
    m_shooter.setInverted(false);
    m_shooterFollower.setInverted(true);
    m_shooterFollower.follow(m_shooter);
    m_shooter.setNeutralMode(NeutralMode.Coast);
    m_shooterFollower.setNeutralMode(NeutralMode.Coast);

    m_shooterFollower.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    m_shooterFollower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    m_shooter.config_kF(0, k_f, k_timeout); 
    m_shooter.config_kP(0, k_p, k_timeout);
    m_shooter.config_kI(0, k_i, k_timeout);
    m_shooter.config_IntegralZone(0, k_iZone, k_timeout);

    // m_amplifier = Shuffleboard.getTab("Drive Tab")
    //   .add("Shooter Amplifier", 0)
    //   .withWidget(BuiltInWidgets.kNumberSlider);
    // m_amplifier.withProperties(Map.of("min", -1000, "max", 1000));
  }

  public double getShooterSpeed() {
    return m_shooter.getSelectedSensorVelocity();
  }

  public double getShooterSetpoint() {
    return m_shooterSetpoint;
  }

  public void setLow(boolean isLow) {
    m_isLow = isLow;
  }
  public boolean getLow() {
    return m_isLow;
  }

  public void setShooterPower(double power) {
    m_shooter.set(ControlMode.PercentOutput, power);
    m_backWheel.set(ControlMode.PercentOutput, power == 0 ? 0 : -0.5); //0.5
    if(power == 0) m_shooterSetpoint = 0;
  }

  public void setShooterSpeed(double speed) {
    m_shooter.set(ControlMode.Velocity, speed);
    System.out.println(String.format("speed=%f", speed));
    m_shooterSetpoint = speed;
  }
  public void setBackWheelPower(double power) {
    m_backWheel.set(ControlMode.PercentOutput, -power);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterSpeed", m_shooter.getSelectedSensorVelocity());
  }

  public void setAdjustment(double speed) {
    m_shooter.set(ControlMode.Velocity, speed + m_adjustment);  
  }
}
