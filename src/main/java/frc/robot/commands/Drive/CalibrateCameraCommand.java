// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.PiCamera.PiCamera;
import frc.PiCamera.PiCamera.PiCameraRegion;
import frc.PiCamera.PiCamera.PiCameraRegions;
import frc.lib.Camera;
import frc.lib.Camera.CameraData;
import frc.robot.PositionTracker;
import frc.robot.PositionTracker.PositionContainer;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrateCameraCommand extends CommandBase {
  DriveSubsystem m_driveSusbsytem;
  double m_speed;
  PositionTracker m_positionTracker;
  Camera m_camera;
  public CalibrateCameraCommand(Camera camera, DriveSubsystem driveSubsystem, double speed) {
    m_camera = camera;
    m_driveSusbsytem = driveSubsystem;
    m_speed = speed;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_positionTracker = m_driveSusbsytem.getPosTracker();
    m_driveSusbsytem.setSpeed(-m_speed, -m_speed);
    m_positionTracker.setXY(0, 3.667);
    m_positionTracker.setAngle(-90);
    m_positionTracker.startPosUpdate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CameraData data = m_camera.createData();
    PiCameraRegions regions = data.m_regions;
    int topRegion = 0;
    if(regions.GetRegionCount() > 0) {
      for(int i = 1; i < regions.GetRegionCount(); i++) {
        if(regions.GetRegion(i).m_bounds.m_top > regions.GetRegion(topRegion).m_bounds.m_top) {
          topRegion = i;
        }
      }
    }
    PiCameraRegion top = regions.GetRegion(topRegion);
    PositionContainer pos = m_positionTracker.getPos();
    System.out.println(String.format("Pos X, %f, Pos Y, %f, Top, %d", pos.x, pos.y, top.m_bounds.m_top));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_positionTracker.stopPosUpdate();
    m_driveSusbsytem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
