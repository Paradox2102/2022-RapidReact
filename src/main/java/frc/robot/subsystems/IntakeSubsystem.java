// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Logger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  TalonSRX m_intake = new TalonSRX(Constants.c.k_intake);
  Solenoid m_deploy = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.c.k_deploy);
  
  public IntakeSubsystem() {
    m_intake.configFactoryDefault();
    m_intake.setInverted(true);
  }

  public void deployIntake(boolean deploy) {
    m_deploy.set(deploy);
  }

  public void runIntake(double power) {
    Logger.Log("IntakeSubsystem", 1, String.format("power=%f", power));
    m_intake.set(ControlMode.PercentOutput, power);

  }

  @Override
  public void periodic() {}
}