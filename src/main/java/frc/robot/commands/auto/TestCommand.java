// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestCommand extends SequentialCommandGroup {
  /*
  0,0,90
  0,5,90
  */
//   private static final Waypoint[] k_path = {
//       new Waypoint(0, 0, Math.toRadians(90)),
//       new Waypoint(0,8, Math.toRadians(90))
//   };

//   private static final Waypoint[] k_reversePath = {
//     new Waypoint(0, 5, Math.toRadians(-90)),
//     new Waypoint(0, 0, Math.toRadians(-90))
// };
/*
0,0,90
-8,8,180
*/
private static final Waypoint[] k_path = {
    new Waypoint(0, 0, Math.toRadians(90)),
    new Waypoint(-8, 8, Math.toRadians(180))
};


  /** Creates a new TestCommand. */
  public TestCommand(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CreatePathCommand(driveSubsystem, k_path, true, false, "Forward 5ft", new PurePursuitData(5)));//,
                // new CreatePathCommand(driveSubsystem, k_reversePath, false, true, "Back 5ft", new PurePursuitData(3, 1.5, 1.5, 100)));
  }
}
