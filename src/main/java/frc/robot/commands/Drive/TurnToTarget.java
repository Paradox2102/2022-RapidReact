// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final double k_deadzone = 50;
  private final double k_minPower = 0.1;
  private double m_distanceFromCenter = 1000;
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
      m_distanceFromCenter = centerLine - center;
      double power = (m_distanceFromCenter) * k_p;
      Logger.Log("TurnToTarget", 1, String.format("power=%f, centerline=%f, center=%f", power, centerLine, center));
      SmartDashboard.putNumber("Camera Top", topRegion.m_bounds.m_top);
      if (Math.abs(power) < k_minPower){
        power = k_minPower * Math.signum(power);
      }
      m_subsystem.setPower(-power, power);
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
    Logger.Log("TurnToTarget", 1, String.format("distanceFromCenter=%f", m_distanceFromCenter));
    return (Math.abs(m_distanceFromCenter) < k_deadzone);
  }
}
