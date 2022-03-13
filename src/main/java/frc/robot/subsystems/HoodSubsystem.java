// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  Servo m_hood1 = new Servo(Constants.c.k_hood1);
  Servo m_hood2 = new Servo(Constants.c.k_hood2);
  
  public HoodSubsystem() {}

  public void setHoodAngle(double angle) {
    double newAngle = (angle + 0.4)/1.8;
    m_hood1.set(newAngle);
    m_hood2.set(newAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
