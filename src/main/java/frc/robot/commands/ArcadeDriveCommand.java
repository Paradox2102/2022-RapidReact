// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {

  DriveSubsystem m_subsystem;

  DoubleSupplier m_getX;
  DoubleSupplier m_getY;

  public ArcadeDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier joystickX, DoubleSupplier joystickY) {
    m_subsystem = driveSubsystem;

    m_getX = joystickX;
    m_getY = joystickY;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_getX.getAsDouble();
    double y = m_getY.getAsDouble();

    x = x * x * x;
    y = y * y * y;
    y *= -1;

    // if (m_getThrottle.getAsDouble() > 0) {
    //   y *= -1;
    // }

    m_subsystem.setPower(y + x, y - x);
    // m_subsystem.setPower(0.25, 0.25);
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
