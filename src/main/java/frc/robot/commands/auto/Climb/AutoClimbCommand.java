// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.Climb;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Climber.ClimbCommand;
import frc.robot.commands.Climber.RotateCommand;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimbCommand extends SequentialCommandGroup {
  /** Creates a new AutoClimbCommand. */
  public AutoClimbCommand(ClimberSubsystem climberSubsystem) {
    addCommands(
      // // Arms up
      // new ParallelDeadlineGroup(new WaitCommand(3), new ClimbCommand(climberSubsystem, () -> {return -0.5;})),
      // // Arms Down
      // new ParallelDeadlineGroup(new WaitCommand(3), new ClimbCommand(climberSubsystem, () -> {return 0.5;})),

      // TO HIGH BAR
      // Arms Up (a little bit)
      new ParallelDeadlineGroup(new WaitCommand(0.25), new ClimbCommand(climberSubsystem, () -> {return -0.9;})),
      // Arms Back
      new RotateCommand(climberSubsystem),
      // new WaitCommand(0.25),
      // Arms up
      new ParallelDeadlineGroup(new WaitCommand(1.75), new ClimbCommand(climberSubsystem, () -> {return -1;})),
      // Arms Foward
      new RotateCommand(climberSubsystem),
      new WaitCommand(0.15),
      // Arms Down (Fast)
      new ParallelDeadlineGroup(new WaitCommand(6.5), new ClimbCommand(climberSubsystem, () -> {return 0.37;})),
      // // Arms Down (Slow)
      // new ParallelDeadlineGroup(new WaitCommand(9), new ClimbCommand(climberSubsystem, () -> {return 0.25;})),

      // new WaitCommand(2),
      // TO TRAVERSAL BAR
      new ParallelDeadlineGroup(new WaitCommand(1), new ClimbCommand(climberSubsystem, () -> {return -0.3;})),
      // Arms Up (a little bit)
      new ParallelDeadlineGroup(new WaitCommand(0.25), new ClimbCommand(climberSubsystem, () -> {return -0.9;})),
      // Arms Back
      new RotateCommand(climberSubsystem),
      // new WaitCommand(0.75),
      // Arms up
      new ParallelDeadlineGroup(new WaitCommand(1.75), new ClimbCommand(climberSubsystem, () -> {return -1;})),
      // Arms Forward
      new RotateCommand(climberSubsystem),
      new WaitCommand(0.15),
      // Arms Down
      new ParallelDeadlineGroup(new WaitCommand(1), new ClimbCommand(climberSubsystem, () -> {return 1;}))
    );
  }
}
