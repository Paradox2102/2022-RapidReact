// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.commands.DisablePositionTrackerCommand;
import frc.robot.commands.SetStateOneBall;
import frc.robot.commands.Drive.TurnToHeadingCommand;
import frc.robot.commands.Scotty.ScottyPowerCommand;
import frc.robot.commands.Shooter.SpinCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ScottySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ED extends ParallelRaceGroup {
  private static final int k_nPoints = 1000;
  private static final double k_dt = 0.020000;
  private static final double k_maxSpeed = 8.000000;
  private static final double k_maxAccel = 8.000000;
  private static final double k_maxDecl = 8.000000;
  private static final double k_maxJerk = 100.000000;
  private static final double k_wheelbase = 1.812500;

  /*1.3,4.891,90
  1.3,20,90
  */
  private static final Waypoint[] k_path = {
      new Waypoint(1.3, 4.891, Math.toRadians(90)),
      new Waypoint(1.3, 20, Math.toRadians(90))
};

  public ED(DriveSubsystem driveSubsystem, ScottySubsystem scottySubsystem, ShooterSubsystem shooterSubsytem, double shooterPower, double scottyPower) {
    addCommands(
      new SpinCommand(shooterSubsytem, shooterPower, false),
      new SequentialCommandGroup(
        new SetStateOneBall(scottySubsystem),
        new WaitCommand(1.5),
        new ParallelDeadlineGroup(new WaitCommand(1),
          new ScottyPowerCommand(scottySubsystem, scottyPower)),
        new CreatePathCommand(driveSubsystem, k_path, true, true, "ED", new PurePursuitData(k_maxSpeed), 0.3),
        new TurnToHeadingCommand(driveSubsystem, 90, 0.6),
        new DisablePositionTrackerCommand(driveSubsystem)
      )
    );
  }
}
