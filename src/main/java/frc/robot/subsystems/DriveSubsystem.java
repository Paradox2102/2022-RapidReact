// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final double k_maxSpeed = 19000; 
  private final double k_p = 0.1;
  private final double k_i = 0.002; 
  private final double k_f = 0.051; 
  private final double k_iZone = 300; 
  private final int k_timeout = 30; 

  public DriveSubsystem() {
    m_rightDriveFollower.setInverted(false);
    m_leftDriveFollower.setInverted(true);
    m_leftDriveFollower.follow(m_leftDrive);
    m_rightDriveFollower.follow(m_rightDrive);
    
    m_leftDrive.setInverted(true);
    m_rightDrive.setInverted(false);
    m_leftDrive.configOpenloopRamp(0.25);
    m_rightDrive.configOpenloopRamp(0.25);

    m_sensors = new Sensor(m_leftCoder, m_rightCoder, m_gyro, 720);
    m_posTracker = new PositionTracker(0, 0, false, m_sensors);
    m_pursuitFollower = new PurePursuit(m_sensors, m_posTracker, (l, r) -> setSpeed(l, r), 50);
    
    m_leftDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, k_timeout); 
    m_rightDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, k_timeout); 
    m_leftDrive.config_kF(0, k_f, k_timeout); 
    m_leftDrive.config_kP(0, k_p, k_timeout);
    m_leftDrive.config_kI(0, k_i, k_timeout);
    m_leftDrive.config_IntegralZone(0, k_iZone, k_timeout);
    m_rightDrive.config_kF(0, k_f, k_timeout); 
    m_rightDrive.config_kP(0, k_p, k_timeout);
    m_rightDrive.config_kI(0, k_i, k_timeout);
    m_rightDrive.config_IntegralZone(0, k_iZone, k_timeout);
    TalonFXSensorCollection leftSensors = m_leftDrive.getSensorCollection(); 
    TalonFXSensorCollection rightSensors = m_rightDrive.getSensorCollection(); 
    leftSensors.setIntegratedSensorPosition(0, k_timeout); 
    rightSensors.setIntegratedSensorPosition(0, k_timeout); 

  }

  public void setSpeed(double left, double right) {
    m_leftDrive.set(TalonFXControlMode.Velocity, left);
    m_rightDrive.set(TalonFXControlMode.Velocity, left);
  }

  public void setPower (double left, double right) {
    m_leftDrive.set(ControlMode.PercentOutput, left);
    m_rightDrive.set(ControlMode.PercentOutput, right);
  }

  public void stop() {
    setPower(0, 0);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("LeftSpeed", m_leftDrive.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("RightSpeed", m_rightDrive.getSelectedSensorVelocity());
    SmartDashboard.putNumber("left", m_leftCoder.getPosition());
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

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("driveSubsystem");
    builder.addDoubleProperty("leftSpeed", ()->{return m_leftDrive.getSelectedSensorVelocity();}, null);
    builder.addDoubleProperty("rightSpeed", ()->{return m_rightDrive.getSelectedSensorVelocity();}, null);
    builder.addDoubleProperty("leftPosition", ()->{return m_leftDrive.getSelectedSensorPosition();}, null); 
    builder.addDoubleProperty("rightPosition", ()->{return m_rightDrive.getSelectedSensorPosition();}, null); 
  }

  public void setPID(double f, double p, double i, double iZone){
    m_leftDrive.config_kF(0, f, k_timeout); 
    m_leftDrive.config_kP(0, p, k_timeout);
    m_leftDrive.config_kI(0, i, k_timeout);
    m_leftDrive.config_IntegralZone(0, iZone, k_timeout);
    m_rightDrive.config_kF(0, f, k_timeout); 
    m_rightDrive.config_kP(0, p, k_timeout);
    m_rightDrive.config_kI(0, i, k_timeout);
    m_rightDrive.config_IntegralZone(0, iZone, k_timeout);
  }
}
