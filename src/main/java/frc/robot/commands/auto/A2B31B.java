// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.commands.DisablePositionTrackerCommand;
import frc.robot.commands.SetStateOneBall;
import frc.robot.commands.Intake.DeployIntakeCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Scotty.ScottyPowerCommand;
import frc.robot.commands.Shooter.SpinCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScottySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A2B31B extends ParallelRaceGroup {
  private static final int k_nPoints = 1000;
private static final double k_dt = 0.020000;
private static final double k_maxSpeed = 7.000000; //4.5
private static final double k_maxAccel = 11.000000; //7
private static final double k_maxDecl = 11.000000; //7 
private static final double k_maxJerk = 100.000000;
private static final double k_wheelbase = 1.812500;
/*
0, 10, 0
3.3, 10, 0
*/
final static Waypoint[] k_firstBall = { new Waypoint(-7.7, 2.25, Math.toRadians(180)), new Waypoint(-10.6, 2.25, Math.toRadians(180)) };

// final static Waypoint[] k_driveShoot = { new Waypoint(-10.6, 2.25, Math.toRadians(0)), new Waypoint(-4, 1, Math.toRadians(-30)) }; //-30
/*
-10.6, 2.25,2.706,2.072,2.197
-4.03,1.722, -30
*/
private static final Waypoint[] k_driveShoot= {
    new Waypoint(-10.6,  2.25, Math.toRadians(0), 2.072, 2.197),
    new Waypoint(-4.03, 1.722, Math.toRadians( -30))
};
/*
-4, 1, 150
-7.2, 10.4, 90
-8.7,21.2, 120
*/
private static final double k_longSpeed = 10; //9

private static final Waypoint[] k_getTwoBalls = {
    new Waypoint(-4, 1, Math.toRadians( 150)),
    new Waypoint(-7.2, 10.4, Math.toRadians( 90)),
    new Waypoint(-8.7, 21.2, Math.toRadians( 120))
};
// private static final Waypoint[] k_getTwoBalls = {
//   new Waypoint(-4,  1, Math.toRadians(-30)),
//   new Waypoint(-6.7,  10.4, Math.toRadians(-90)),
//   new Waypoint(-8.5,  21.5, Math.toRadians(-60))
// };
/*
-8.7,21.2,-60
-3.6, 1.2, -30
// */
// private static final Waypoint[] k_driveShootLong = {
//     new Waypoint(-8.7, 21.2, Math.toRadians(-60)),
//     new Waypoint(-3.6,  1.2, Math.toRadians( -30))
//};
/*
-8.7,21.2,-60,4.785,6.88
// -3.6, 1.2, -30
// */
// private static final Waypoint[] k_driveShootLong = {
//     new Waypoint(-8.7, 21.2, Math.toRadians(-60), 4.785, 6.88),
//     new Waypoint(-3.6,  1.2, Math.toRadians( -30))
//};
/*
-8.7,21.2,-60,4.785,6.88
-3.209,1.33, -20
*/
private static final Waypoint[] k_driveShootLong = {
    new Waypoint(-8.7, 21.2, Math.toRadians(-60), 4.785, 6.88),
    new Waypoint(-3.209, 1.33, Math.toRadians( -20))
};


  public A2B31B(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsytem, ScottySubsystem scottySubsystem, double intakePower, double shooterPower, double scottyPower) {
    addCommands(
      new IntakeCommand(intakeSubsystem, scottySubsystem, intakePower),
      new SpinCommand(shooterSubsytem, () -> false),
      new SequentialCommandGroup(
        new SetStateOneBall(scottySubsystem),
        new CreatePathCommand(driveSubsystem, k_firstBall, true, true, "Backwards get first ball", new PurePursuitData(k_maxSpeed)),
        new CreatePathCommand(driveSubsystem, k_driveShoot, false, false, "Drive up and shoot", new PurePursuitData(k_maxSpeed)),
        new ProxyScheduleCommand(new ScottyPowerCommand(scottySubsystem, scottyPower, Constants.k_threeBallTime)),       
        new CreatePathCommand(driveSubsystem, k_getTwoBalls, false, true, "Pickup two balls", new PurePursuitData(k_longSpeed), 0.3),
        new CreatePathCommand(driveSubsystem, k_driveShootLong, false, false, "Drive Long and Shoot", new PurePursuitData(k_longSpeed), 0.3),
        new ProxyScheduleCommand(new ScottyPowerCommand(scottySubsystem, scottyPower, Constants.k_twoBallTime)),
        new DisablePositionTrackerCommand(driveSubsystem)
      )
    );
  }
}
