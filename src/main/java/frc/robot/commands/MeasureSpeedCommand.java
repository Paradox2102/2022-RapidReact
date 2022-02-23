// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.subsystems.DriveSubsystem;

public class MeasureSpeedCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem; 
  private double k_f = 0; 
  private double k_p = 0; 
  private double k_i = 0; 
  private double k_iZone = 0; 

  /** Creates a new MeasureSpeedCommand. */
  public MeasureSpeedCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Log("MeasureSpeedCommand", 2, "initialize");
    // m_driveSubsystem.setPower(0.5, 0.5); 
    m_driveSubsystem.setSpeed(30, 700); 
    // SmartDashboard.putNumber("k_f", k_f);
    // SmartDashboard.putNumber("k_p", k_p);
    // SmartDashboard.putNumber("k_i", k_i);
    // SmartDashboard.putNumber("k_iZone", k_iZone);
    // k_f = SmartDashboard.getNumber("k_f", 0); 
    // k_p = SmartDashboard.getNumber("k_p", 0); 
    // k_i = SmartDashboard.getNumber("k_i", 0); 
    // k_iZone = SmartDashboard.getNumber("k_iZone", 0);
    // m_driveSubsystem.setPID(k_f, k_p, k_i, k_iZone); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.Log("MeasureSpeedCommand", 2, "end");
    m_driveSubsystem.stop(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
