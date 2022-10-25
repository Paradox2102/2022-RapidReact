// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Logger;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinCommand extends CommandBase {

  ShooterSubsystem m_shooterSubsystem; 
  final double k_nearPower = 8000;
  final double k_nearBackPower = 10000;

  final double k_farPower = 7000;
  final double k_farBackPower = 12000; 

  BooleanSupplier m_farShot;

  public SpinCommand(ShooterSubsystem shooterSubsytem, double frontPower, double backPower) {
    this(shooterSubsytem, () -> false);
  }

  public SpinCommand(ShooterSubsystem shooterSubsystem, BooleanSupplier farShot){
    m_shooterSubsystem = shooterSubsystem;
    m_farShot = farShot;

    addRequirements(m_shooterSubsystem);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Log("Spin Up Commandxx", 1, "Initialized");
    // m_shooterSubsystem.setLow(m_shootLow);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean farShot 
    = m_farShot.getAsBoolean();
    m_shooterSubsystem.setShooterSpeed(farShot ? k_farPower : k_nearPower);
    m_shooterSubsystem.setBackWheelSpeed(farShot ? k_farBackPower : k_nearBackPower);
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Spin Up Command End");
    m_shooterSubsystem.setShooterPower(0);
    m_shooterSubsystem.setBackWheelPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
