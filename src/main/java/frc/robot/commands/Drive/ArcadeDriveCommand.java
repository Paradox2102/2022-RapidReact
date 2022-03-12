// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.DriveHelper;
import frc.lib.DriveSignal;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {

  DriveSubsystem m_subsystem;

  DoubleSupplier m_getX;
  DoubleSupplier m_getY;

  DoubleSupplier m_getThrottle;

  private static final double k_maxSpeed = 19000;
  private DriveHelper m_driveHelper = new DriveHelper();

  enum driveTypes{
    normal, halfPower, speedControl, speedControlHalfSpeed, curvature, curvatureHalfSpeed, curvatureLSM
  }
  private SendableChooser<driveTypes> m_chooser = new SendableChooser<>();

  public ArcadeDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier getThrottle) {
    m_subsystem = driveSubsystem;

    m_getX = joystickX;
    m_getY = joystickY;

    m_getThrottle = getThrottle;

    addRequirements(driveSubsystem);
    m_chooser.setDefaultOption("normal", driveTypes.normal);
    m_chooser.addOption("halfPower", driveTypes.halfPower);
    m_chooser.addOption("speedControl", driveTypes.speedControl);
    m_chooser.addOption("speedControlHalfSpeed",driveTypes.speedControlHalfSpeed);
    m_chooser.addOption("curvature", driveTypes.curvature);
    m_chooser.addOption("curvatureHalfSpeed", driveTypes.curvatureHalfSpeed);
    m_chooser.addOption("curvature with Low Speed Mode", driveTypes.curvatureLSM);
    SmartDashboard.putData("drive mode", m_chooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_getX.getAsDouble();
    double y = m_getY.getAsDouble();

    x = x * x * x / 2;
    y = y * y * y;
    y *= -1;

    if (m_getThrottle.getAsDouble() > 0) {
      y *= -1;
    }

    // m_subsystem.setPower(y + x, y - x);
    // m_subsystem.setPower(0.25, 0.25);
    switch (m_chooser.getSelected()) {

      case normal:
        m_subsystem.setPower(y + x, y - x);
        break;
      case halfPower:
        x = x / 2;
        m_subsystem.setPower(y + x, y - x);
        break;
      case speedControl:
        x = x * k_maxSpeed;
        y = y * k_maxSpeed;
        m_subsystem.setSpeed(y + x, y - x);
        break;
      case speedControlHalfSpeed:
        x = x * k_maxSpeed / 2;
        y = y * k_maxSpeed;
        m_subsystem.setSpeed(y + x, y - x);
        break;
      case curvature: {
        DriveSignal ds = m_driveHelper.cheesyDrive(y, x, false);
        m_subsystem.setSpeed(ds.leftMotor * k_maxSpeed, ds.rightMotor * k_maxSpeed);
      }
        break;
      case curvatureHalfSpeed: {
        double speedVariable = .8;
        DriveSignal ds = m_driveHelper.cheesyDrive(y * speedVariable, x, false);
        m_subsystem.setSpeed(ds.leftMotor * k_maxSpeed, ds.rightMotor * k_maxSpeed);
      }
        break;
      case curvatureLSM: {
        double speedVariable = 1;
        DriveSignal ds = m_driveHelper.cheesyDrive(y * speedVariable, x, false);
        DriveSignal dsLow = m_driveHelper.cheesyDrive(y * speedVariable, x, true);
        double leftMotor;
        double rightMotor;
        double lowSpeedThreshhold = .01;
        if (Math.abs(y) < lowSpeedThreshhold) {
          leftMotor = dsLow.leftMotor * Math.abs(y) / lowSpeedThreshhold
              + ds.leftMotor * (lowSpeedThreshhold - Math.abs(y)) / lowSpeedThreshhold;
          rightMotor = dsLow.rightMotor * Math.abs(y) / lowSpeedThreshhold
              + ds.rightMotor * (lowSpeedThreshhold - Math.abs(y)) / lowSpeedThreshhold;
        } else {
          leftMotor = ds.leftMotor;
          rightMotor = ds.rightMotor;
        }
        m_subsystem.setSpeed(leftMotor * k_maxSpeed, rightMotor * k_maxSpeed);
      }
        break;
    }
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
