// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.PiCamera.Logger;
import frc.PiCamera.PiCamera.PiCameraRegion;
import frc.lib.Camera;
import frc.lib.Camera.CameraData;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTarget extends CommandBase {
  DriveSubsystem m_subsystem;
  Camera m_camera;
  private final double k_p = .0004;
  /** Creates a new TurnToTarget. */
  public TurnToTarget(DriveSubsystem driveSubsystem, Camera camera) {
    m_subsystem = driveSubsystem;
    m_camera = camera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Log("TurnToTarget", 1, "initialize");
    m_camera.toggleLights(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CameraData m_cameraData = m_camera.createData();
    if (m_cameraData.canSee()) {
      PiCameraRegion topRegion = m_cameraData.getTopMostRegion();
      double center = (topRegion.m_bounds.m_left + topRegion.m_bounds.m_right) / 2.0;
      double centerLine = m_cameraData.centerLine();
      double power = (centerLine - center) * k_p;
      m_subsystem.setPower(-power, power);
      // if (center < centerLine){
      //   m_subsystem.setPower(-power, power);
      // } else if (center > centerLine){
      //   m_subsystem.setPower(power, -power);
      // }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_camera.toggleLights(false);
    m_subsystem.stop();
    Logger.Log("TurnToTarget", 1, "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
