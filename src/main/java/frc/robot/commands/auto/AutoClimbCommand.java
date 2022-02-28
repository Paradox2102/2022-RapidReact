// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimbCommand extends SequentialCommandGroup {
  /** Creates a new AutoClimbCommand. */
  public AutoClimbCommand(ClimberSubsystem climberSubsystem) {
    addCommands(
      new ClimbCommand(climberSubsystem),
      new WaitCommand(1.5),
      new ClimbCommand(climberSubsystem),
      new WaitCommand(0.5),
      new ClimbCommand(climberSubsystem),
      new WaitCommand(3),
      new ClimbCommand(climberSubsystem)
    );
  }
}
