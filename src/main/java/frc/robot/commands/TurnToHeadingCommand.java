// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToHeadingCommand extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  double m_angle;
  boolean m_clockwise;
  double m_power;
  // final double k_turnP = ;
  public TurnToHeadingCommand(DriveSubsystem driveSubsystem,double angle, boolean clockwise, double power) {
    m_driveSubsystem = driveSubsystem;
    m_angle = angle;
    m_clockwise = clockwise;
    m_power = power;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_driveSubsystem.getYaw();
    double delta = m_clockwise ? (m_angle - angle)%360 : ((angle - m_angle)%360)-360;
    double power = delta * m_power;
    m_driveSubsystem.setPower(power, -power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
