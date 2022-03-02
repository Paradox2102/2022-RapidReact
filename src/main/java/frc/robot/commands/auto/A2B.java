+// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScottyPowerCommand;
import frc.robot.commands.SpinCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScottySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A2B extends ParallelCommandGroup {
  private static final int k_nPoints = 1000;
private static final double k_dt = 0.020000;
private static final double k_maxSpeed = 4.500000;
private static final double k_maxAccel = 7.000000;
private static final double k_maxDecl = 7.000000;
private static final double k_maxJerk = 100.000000;
private static final double k_wheelbase = 1.812500;
/*
0, 10, 0
3.3, 10, 0
*/
final static Waypoint[] k_firstBall = { 
    new Waypoint(-7.7, 2.25, Math.toRadians(180)), 
    new Waypoint(-11.1, 2.25, Math.toRadians(180)) };

final static Waypoint[] k_driveShoot = { 
    new Waypoint(-11.1, 2.25, Math.toRadians(0)), 
    new Waypoint(-4, 1, Math.toRadians(-30)) };

  public A2B(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsytem, ScottySubsystem scottySubsystem, double intakePower, double shooterPower, double scottyPower) {
    addCommands(
      new IntakeCommand(intakeSubsystem, scottySubsystem, intakePower),
      new SpinCommand(shooterSubsytem, shooterPower),
      new SequentialCommandGroup(
        new CreatePathCommand(driveSubsystem, k_firstBall, true, true, "Backwards get first ball", new PurePursuitData(k_maxSpeed)),
        new CreatePathCommand(driveSubsystem, k_driveShoot, false, false, "Drive up and shoot", new PurePursuitData(k_maxSpeed)),
        new ScottyPowerCommand(scottySubsystem, scottyPower)
      )
    );
  }
}