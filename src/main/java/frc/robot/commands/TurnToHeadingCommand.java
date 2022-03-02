// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.Normalizer;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToHeadingCommand extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  double m_angle;
  boolean m_clockwise;
  double m_power;
  double m_targetAngle; 
  // final double k_turnP = ;
  public TurnToHeadingCommand(DriveSubsystem driveSubsystem,double angle, double power) {
    m_driveSubsystem = driveSubsystem;
    m_angle = angle;
    m_power = power;
    addRequirements(driveSubsystem);
  }

  //This function takes an angle and returns an angle between +180 and -180 
  public double normalize(double angle) {
    
    angle = angle%360;

    if (angle > 180){
      angle -=360; 
    }
    else 
    if (angle < -180){
      angle +=360; 
    }
    return angle;
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double yaw = m_driveSubsystem.getYaw(); 
    double delta = normalize (m_angle - yaw);

    m_targetAngle = yaw + delta; 
    
    if (delta > 0) { 
      m_clockwise = false;
      m_driveSubsystem.setPower(-m_power, m_power);  
    }
    else {
      m_clockwise = true; 
      m_driveSubsystem.setPower(m_power, -m_power); 
    }
    Logger.Log("TurnToHeadingCommand", 3, String.format("yaw = %f, delta = %f, clockwise = %b, m_angle = %f", yaw, delta, m_clockwise, m_angle)); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double delta = m_clockwise ? (m_angle - angle)%360 : ((angle - m_angle)%360)-360;
    // double power = delta * m_power;

    // m_driveSubsystem.setPower(power, -power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double yaw = m_driveSubsystem.getYaw();
    Logger.Log("TurneToHeadingCommand", 1, String.format("yaw = %f, targetAngle = %f", yaw, m_targetAngle)); 

    if(m_clockwise){
      if (yaw <= m_targetAngle) {
        return true; 
      }
    }
    else {
      if (yaw >= m_targetAngle) {
        return true; 
      }
    }

    return false;
  }
}
