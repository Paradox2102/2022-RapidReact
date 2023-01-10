// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScottySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullTestRun2023 extends ParallelCommandGroup {
  private static final double k_dt = 0.020000;
  private static final double k_maxSpeed = 7.000000;
  private static final double k_maxAccel = 11.000000;
  private static final double k_maxDecl = 11.000000;
  private static final double k_maxJerk = 100.000000;
  private static final double k_wheelbase = 1.812500;
  /*
0,0,90
0,18.4,90
*/
static final Waypoint[] k_pathfoward = {
  new Waypoint(0, 0, Math.toRadians(90)),
  new Waypoint(0, 18.4, Math.toRadians(90))};

static final Waypoint[] k_pathbackward = {
  new Waypoint(0, 18.4, Math.toRadians(90)),
  new Waypoint(0, 0, Math.toRadians(90))};

  public FullTestRun2023(DriveSubsystem driveSubsystem) {
    addCommands(
      new SequentialCommandGroup(
        new WaitCommand(1),
        new CreatePathCommand(driveSubsystem, k_pathfoward, true, true, "Forwards get first ball", new PurePursuitData(k_maxSpeed)),
        new WaitCommand(1),
        new CreatePathCommand(driveSubsystem, k_pathbackward, false, false, "Backwards to grid", new PurePursuitData(k_maxSpeed))
        // new WaitCommand(1)
      )
    );
  }
};