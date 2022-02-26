// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
public class TwoBallAuto extends ParallelRaceGroup {
  
  private static final int k_nPoints = 1000;
private static final double k_dt = 0.020000;
private static final double k_maxSpeed = 4.500000;
private static final double k_maxAccel = 7.000000;
private static final double k_maxDecl = 7.000000;
private static final double k_maxJerk = 100.000000;
private static final double k_wheelbase = 1.812500;
/*
3.75,7.75, 46.52
6,10, 46.52
*/
private static final Waypoint[] k_firstBall = {
    new Waypoint(3.75, 7.75, Math.toRadians( 46.52)),
    new Waypoint(6, 10, Math.toRadians( 46.52))
};
/*
6,10, 226.52
1, 3, 248
*/
private static final Waypoint[] k_driveShoot = {
    new Waypoint(6, 10, Math.toRadians( 226.52)),
    new Waypoint(1.2,  3.5, Math.toRadians( 248))
};
  public TwoBallAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ScottySubsystem scottySubsystem, ShooterSubsystem shooterSubsytem, double intakePower, double shooterPower, double scottyPower) {
    addCommands(
      new IntakeCommand(intakeSubsystem, scottySubsystem, intakePower),
      new SpinCommand(shooterSubsytem, shooterPower),
      new SequentialCommandGroup(
        new CreatePathCommand(driveSubsystem, k_firstBall, true, true, "Drive to first ball", new PurePursuitData(k_maxSpeed)),
        new CreatePathCommand(driveSubsystem, k_driveShoot, false, false, "Drive to hub", new PurePursuitData(k_maxSpeed)),
        new FireAutoCommand(scottySubsystem, scottyPower, 1000)
      )
    );
  }
}
