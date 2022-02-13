// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  
  TalonFX m_leftDrive = new TalonFX(Constants.k_driveLeft);
  TalonFX m_leftDriveFollower = new TalonFX(Constants.k_driveLeftFollower);

  TalonFX m_rightDrive = new TalonFX(Constants.k_driveRight);
  TalonFX m_rightDriveFollower = new TalonFX(Constants.k_driveRightFollower);

  PIDController m_leftController;
  PIDController m_rightController;

  public DriveSubsystem() {
    m_rightDriveFollower.setInverted(false);
    m_leftDriveFollower.setInverted(true);
    m_leftDriveFollower.follow(m_leftDrive);
    m_rightDriveFollower.follow(m_rightDrive);
    
    m_leftDrive.setInverted(true);
    m_rightDrive.setInverted(false);
    m_leftDrive.configOpenloopRamp(2d);
    m_rightDrive.configOpenloopRamp(2d);
  }

  public void setPower (double left, double right) {
    m_leftDrive.set(ControlMode.PercentOutput, left);
    m_rightDrive.set(ControlMode.PercentOutput, right);
  }

  @Override
  public void periodic() {

  }
}
