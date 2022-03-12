// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.PiCamera.PiCamera.PiCameraRegion;
import frc.lib.Camera;
import frc.lib.Logger;
import frc.lib.Camera.CameraData;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimToTargetCommand extends CommandBase {

  DriveSubsystem m_driveSubsystem;
  ShooterSubsystem m_shooterSubsytem;
  Camera m_camera;
  double m_speed;
  final int k_deadzone = 100;
  final double k_p = 0.003;

  public AimToTargetCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, Camera camera, double speed) {
    m_driveSubsystem = driveSubsystem;
    m_shooterSubsytem = shooterSubsystem;
    m_camera = camera;
    m_speed = speed;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.toggleLights(true);
    Logger.Log("AimToTargetCommand", 1, "Initialize");
    // m_driveSubsystem.setSpeed(-800, right);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CameraData data = m_camera.createData();

    if(data.m_regions.GetRegionCount() > 0) {
      PiCameraRegion top = data.getTopRegion();
      double middle = (top.m_bounds.m_right + top.m_bounds.m_left)/2;
      double speed = m_speed * Math.abs(data.centerLine() - middle) * k_p;
      if(data.centerLine() > middle + k_deadzone) {
        m_driveSubsystem.setSpeed(-speed, speed);
      } else if(data.centerLine() < middle - k_deadzone) {
        m_driveSubsystem.setSpeed(speed, -speed);
      } else {
        m_driveSubsystem.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
    m_camera.toggleLights(false);
    Logger.Log("AimToTargetCommand", 1, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
