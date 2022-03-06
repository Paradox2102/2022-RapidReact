// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DefaultScottyCommand;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MeasureSpeedCommand;
import frc.robot.commands.RatchetCommand;
import frc.robot.commands.ScottyPowerCommand;
import frc.robot.commands.ServoThrottleCommand;
import frc.robot.commands.SpinCommand;
import frc.robot.commands.WinchCommand;
import frc.robot.commands.auto.TestCommand;
import frc.robot.commands.auto.F4E;
import frc.robot.commands.auto.ED;
import frc.robot.commands.auto.A2B3;
import frc.robot.commands.auto.A2B31B;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScottySubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Constants m_constants = Constants.getInstance();

  DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  ScottySubsystem m_scottySubsystem = new ScottySubsystem();
  IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  Joystick m_stick = new Joystick(0);
  Joystick m_climbStick = new Joystick(1);
  // Joystick m_calibStick = new Joystick(5);

  // Driver 1
  JoystickButton m_intake = new JoystickButton(m_stick, 1);
  JoystickButton m_outake = new JoystickButton(m_stick, 2);
  // JoystickButton m_testSpeed = new JoystickButton(m_stick, 10); 
  // Driver 2
  // JoystickButton m_climb = new JoystickButton(m_climbStick, 11);
  JoystickButton m_ratchetOn = new JoystickButton(m_climbStick, 5);
  JoystickButton m_ratchetOff = new JoystickButton(m_climbStick, 3);
  JoystickButton m_winch = new JoystickButton(m_climbStick, 6);
  JoystickButton m_reverseWinch = new JoystickButton(m_climbStick, 4);
  // JoystickButton m_deployIntake = new JoystickButton(m_climbStick, 3);
  JoystickButton m_fire = new JoystickButton(m_climbStick, 1);
  // JoystickButton m_reverseScotty = new JoystickButton(m_climbStick, 6);
  JoystickButton m_spinUp = new JoystickButton(m_climbStick, 2);
  // Calib
  // JoystickButton m_testPath = new JoystickButton(m_calibStick, 2);
  // JoystickButton m_calibRatchet = new JoystickButton(m_calibStick, 3);
  // JoystickButton m_deployIntake = new JoystickButton(m_calibStick, 4);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    // SmartDashboard.putData(new PowerDistribution());
    SmartDashboard.putData(m_driveSubsystem);
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, () -> m_stick.getX(), 
        () -> m_stick.getY(), () -> m_stick.getThrottle()));
    m_scottySubsystem.setDefaultCommand(new DefaultScottyCommand(m_scottySubsystem, 0.3));

    m_chooser.addOption("A2B31B (Four)", new A2B31B(m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_scottySubsystem, 0.7, 7100, 0.35));
    m_chooser.addOption("F4E (Two)", new F4E(m_driveSubsystem, m_intakeSubsystem, m_scottySubsystem, m_shooterSubsystem, 0.7, 7100, 0.5));
    m_chooser.addOption("ED (One)", new ED(m_driveSubsystem, m_scottySubsystem, m_shooterSubsystem, 7100, 0.5));
    m_chooser.addOption("A2B3 (Three)", new A2B3(m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_scottySubsystem, 0.7, 7100, 0.35));
 
    SmartDashboard.putData(m_chooser);

    // ShuffleboardTab driverTab = Shuffleboard.getTab("Drive Tab");
    // driverTab.addCamera("Camera Viewer", "Front Camera", "http://10.21.2.2:1181/?action=stream").withSize(5, 4).withPosition(1, 1);
    // driverTab.add(CameraServer.getServer().getSource());
  }
  // m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, () -> m_stick.getX(),
  //       () -> (-m_stick.getY() - m_velocityStick.getY()), () -> m_stick.getThrottle()));

  private void configureButtonBindings() {
    // Driver 1
    // m_spinUp.toggleWhenPressed(new SpinCommand(m_shooterSubsystem, () -> m_stick.getThrottle()));
    m_intake.whileHeld(new IntakeCommand(m_intakeSubsystem, m_scottySubsystem, 0.70));
    m_outake.whileHeld(new IntakeCommand(m_intakeSubsystem, m_scottySubsystem, -0.60));
    // m_testSpeed.whileHeld(new MeasureSpeedCommand(m_driveSubsystem)); 
    // Driver 2
    // m_climb.whenPressed(new ClimbCommand(m_climberSubsystem));
    m_ratchetOn.whenPressed(new RatchetCommand(m_climberSubsystem, true));
    m_ratchetOff.whenPressed(new RatchetCommand(m_climberSubsystem, false));
    // m_fire.whileHeld(new ScottyPowerCommand(m_scottySubsystem, 0.4));
    m_fire.whileHeld(new FireCommand(m_scottySubsystem, m_shooterSubsystem, 0.3));
    m_winch.whileHeld(new WinchCommand(m_climberSubsystem, 0.85));
    m_reverseWinch.whileHeld(new WinchCommand(m_climberSubsystem, -0.5));

    // m_reverseScotty.whileHeld(new ScottyPowerCommand(m_scottySubsystem, -0.4));
    // m_spinUp.toggleWhenPressed(new SpinCommand(m_shooterSubsystem, () -> m_climbStick.getThrottle()));
    m_spinUp.toggleWhenPressed(new SpinCommand(m_shooterSubsystem, 7300));
    // Calib Driver
    // m_testPath.toggleWhenPressed(new A2B31B(m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_scottySubsystem, 0.7, 7100, 0.3));
    // m_calibRatchet.toggleWhenPressed(new ServoThrottleCommand(m_climberSubsystem, () -> m_calibStick.getThrottle()));
    // m_deployIntake.toggleWhenPressed(new DeployIntakeCommand(m_intakeSubsystem));
    // m_testPath.toggleWhenPressed(new TwoBallAuto(m_driveSubsystem, m_intakeSubsystem, m_scottySubsystem, m_shooterSubsystem, 0.7, 0.52, 0.6));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
