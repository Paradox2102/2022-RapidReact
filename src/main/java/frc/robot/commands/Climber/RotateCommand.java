package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class RotateCommand extends InstantCommand{
    ClimberSubsystem m_climberSubsystem;
    public RotateCommand(ClimberSubsystem climberSubsystem) {
      m_climberSubsystem = climberSubsystem;
      addRequirements(climberSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_climberSubsystem.toggleRotate();
    }
  
  }
