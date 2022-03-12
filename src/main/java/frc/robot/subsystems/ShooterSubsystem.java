// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.PiCamera.PiCamera.PiCameraRegion;
import frc.PiCamera.PiCamera.PiCameraRegions;
import frc.lib.Camera;
import frc.lib.Camera.CameraData;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  TalonFX m_shooter = new TalonFX(Constants.c.k_shooter);
  TalonFX m_shooterFollower = new TalonFX(Constants.c.k_shooterFollower);

  Servo m_hood = new Servo(Constants.c.k_hood);

  double k_f = 0.061;
  double k_p = 0.25;
  double k_i = 0.001;
  double k_iZone = 100;
  int k_timeout = 30;

  double m_shooterSetpoint = 0;
  SimpleWidget m_amplifier;

  public ShooterSubsystem() {
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

  public double getDistanceFromTarget(CameraData data) {

    return 0;
  }

  public PiCameraRegion getTopRegion(CameraData data) {
    PiCameraRegions regions = data.m_regions;
    int topRegion = 0;
    if(regions.GetRegionCount() > 0) {
      for(int i = 0; i < regions.GetRegionCount(); i++) {
        if(regions.GetRegion(i).m_bounds.m_top < regions.GetRegion(topRegion).m_bounds.m_top) {
          topRegion = i;
        }
      }
    }
    return regions.GetRegion(topRegion);
  }

  public void setShooterPower(double power) {
    m_shooter.set(ControlMode.PercentOutput, power);
  }

  public void setShooterSpeed(double speed) {
    m_shooter.set(ControlMode.Velocity, speed);
    m_shooterSetpoint = speed;
  }

  public void setHoodAngle(double angle) {
    m_hood.set(angle);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("ShooterSpeed", m_shooter.getSelectedSensorVelocity());
  }
}
