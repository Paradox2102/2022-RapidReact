// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.pathfinder.Pathfinder.Path;
import frc.robot.Constants;
import frc.robot.PositionTracker;
import frc.robot.PurePursuit;
import frc.robot.Sensor;

public class DriveSubsystem extends SubsystemBase {
  
  TalonFX m_leftDrive = new TalonFX(Constants.k_driveLeft);
  TalonFX m_leftDriveFollower = new TalonFX(Constants.k_driveLeftFollower);
  CANCoder m_leftCoder = new CANCoder(Constants.k_driveLeft);

  TalonFX m_rightDrive = new TalonFX(Constants.k_driveRight);
  TalonFX m_rightDriveFollower = new TalonFX(Constants.k_driveRightFollower);
  CANCoder m_rightCoder = new CANCoder(Constants.k_driveRight);

  PigeonIMU m_gyro = new PigeonIMU(0);

  PIDController m_leftController;
  PIDController m_rightController;

  private final PositionTracker m_posTracker;
  public final PurePursuit m_pursuitFollower;
  private final Sensor m_sensors;

  public DriveSubsystem() {
    m_rightDriveFollower.setInverted(false);
    m_leftDriveFollower.setInverted(true);
    m_leftDriveFollower.follow(m_leftDrive);
    m_rightDriveFollower.follow(m_rightDrive);
    
    m_leftDrive.setInverted(true);
    m_rightDrive.setInverted(false);
    m_leftDrive.configOpenloopRamp(1d);
    m_rightDrive.configOpenloopRamp(1d);

    m_sensors = new Sensor(m_leftCoder, m_rightCoder, m_gyro, 720);
    m_posTracker = new PositionTracker(0, 0, false, m_sensors);
    m_pursuitFollower = new PurePursuit(m_sensors, m_posTracker, (l, r) -> setSpeed(l, r), 50);

  }

  public void setSpeed(double left, double right) {

  }

  public void setPower (double left, double right) {
    m_leftDrive.set(ControlMode.PercentOutput, left);
    m_rightDrive.set(ControlMode.PercentOutput, right);
  }

  @Override
  public void periodic() {

  }

  //Pure Pursuit
  /*
   * This function loads a specified path and the starts the following
   */
  public void startPath(Path path, boolean isReversed, boolean setPosition) {
    m_pursuitFollower.loadPath(path, isReversed, true, setPosition);
    m_pursuitFollower.startPath();
  }

  /*
   * This function ends the currently followed path (if any)
   */
  public void endPath() {
    m_pursuitFollower.stopFollow();
  }
  /*
   * This function returns true if the current path has completed
   */
  public boolean isPathFinished() {
    return (m_pursuitFollower.isFinished());
  }

    /*
   * Sets the speed in feet per second
   */
  public void setSpeedFPS(double leftSpeed, double rightSpeed)
  {
    // // Change speed from FPS to -1 to 1 range
    // leftSpeed = leftSpeed * k_ticksPerFoot / k_maxSpeed;
    // rightSpeed =  rightSpeed * k_ticksPerFoot / k_maxSpeed;

    // // Logger.Log("DriveSubsystem", 1, String.format("setSpeedFPS: left=%f, right=%f", leftSpeed, rightSpeed));

    // setSpeed(leftSpeed, rightSpeed);
  }
}
