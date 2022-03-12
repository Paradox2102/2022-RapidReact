// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.PiCamera.PiCamera.PiCameraRegion;
import frc.lib.Camera;
import frc.lib.Logger;
import frc.lib.Camera.CameraData;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootByDistanceCommand extends CommandBase {
  final int[][] table = {
    // dist, speed, angle
    {1, 2, 3},
    {4, 5, 6}
  };
  ShooterSubsystem m_subsystem;
  double m_speed;
  Camera m_camera;
  public ShootByDistanceCommand(ShooterSubsystem subsystem, Camera camera, double defaultSpeed) {
    m_subsystem = subsystem;
    m_speed = defaultSpeed;
    m_camera = camera;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Log("Shoot Distance Command", 1, "Initialized");
    m_camera.toggleLights(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CameraData data = m_camera.createData();
    //             Speed, Angle
    double[] ans = { m_speed, 0d };
    if(data != null && data.canSee() && data.m_regions.GetRegionCount() > 0) {
      double distance = data.getDistanceFromTarget();
      // SmartDashboard.putNumber("Distance", distance);

      // Clamp the speed and angle to the maximum and minimum values
      if(distance < table[0][0]) {
        ans[0] = table[0][1];
        ans[1] = table[0][2];
      } else if(distance > table[table.length][0]) {
        ans[0] = table[table.length][1];
        ans[1] = table[table.length][2];
      } else { // Interpolate between the two closest values
        for(int i = 1; i < table.length; i++) {
          if(distance >= table[i][0]) {
            ans[0] = interpolate(distance, table[i-1][0], table[i][0], table[i-1][1], table[i][1]);
            ans[1] = interpolate(distance, table[i-1][0], table[i][0], table[i-1][2], table[i][2]);
            break;
          }
        }
      }
    }

    // SmartDashboard.putNumber("Set Shooter Speed", ans[0]);
    // SmartDashboard.putNumber("Set Angle", ans[1]);
    m_subsystem.setShooterSpeed(ans[0]);
    m_subsystem.setHoodAngle(ans[1]);
  }

  private double interpolate(double x, double x1, double x2, double y1, double y2) {
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.Log("Shoot Distance Command", 1, "Ended");
    m_camera.toggleLights(false);
    m_subsystem.setShooterPower(0);
    m_subsystem.setHoodAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
