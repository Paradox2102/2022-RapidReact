// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.pathfinder.Pathfinder.Waypoint;
// import frc.robot.PurePursuit.PathConfigs;
// import frc.robot.commands.Auto.CreatePathCommand;
// import frc.robot.subsystems.DriveSubsystemSPARKMAX;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class FourBallAuto extends SequentialCommandGroup {

//   final static Waypoint[] k_firstBall = { new Waypoint(-7.7, 2.25, Math.toRadians(180)),
//     new Waypoint(-11.6, 2.25, Math.toRadians(180)) };

//   final static Waypoint[] k_bigTurn = { new Waypoint(0, 0, Math.toRadians(90)),
//     new Waypoint(-10, 10, Math.toRadians(180)) };

//   final static Waypoint[] k_driveShoot = { new Waypoint(-11.6, 2.25, Math.toRadians(0)),
//     new Waypoint(-3.3, 1.6, Math.toRadians(-30)) };

//   final static Waypoint[] k_secondBall = { new Waypoint(-3.3, 1.6, Math.toRadians(150)),
//   new Waypoint(-7.36, 9.2, Math.toRadians(90)), new Waypoint(-9, 22.4, Math.toRadians(120)) };

//   final static Waypoint[] k_final = { new Waypoint(-9, 22.4, Math.toRadians(-60)),
//     new Waypoint(-3.3, 1.6, Math.toRadians(-30)) };

//   public FourBallAuto(DriveSubsystemSPARKMAX driveSubsystem) {
//     System.out.println("Path started");
//     addCommands(
//       new CreatePathCommand(driveSubsystem, k_firstBall, PathConfigs.slow, true, true, true),
//       new CreatePathCommand(driveSubsystem, k_driveShoot, PathConfigs.fastAccel, false, false, true),
//       new WaitCommand(1),
//       new CreatePathCommand(driveSubsystem, k_secondBall, PathConfigs.fast, true, false, true),
//       new CreatePathCommand(driveSubsystem, k_final, PathConfigs.fast, false, false, true)
//     );
//   }
// }
