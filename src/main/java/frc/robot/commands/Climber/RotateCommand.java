package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class RotateCommand extends InstantCommand{
    ClimberSubsystem m_climberSubsystem;
    Boolean m_rotate;
    public RotateCommand(ClimberSubsystem climberSubsystem) {
      m_climberSubsystem = climberSubsystem;
      addRequirements(climberSubsystem);
    }
    public RotateCommand(ClimberSubsystem climberSubsystem, boolean rotate) {
      m_climberSubsystem = climberSubsystem;
      m_rotate = rotate;
      addRequirements(climberSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if(m_rotate != null) {
        m_climberSubsystem.toggleRotate(m_rotate.booleanValue());
      } else {
        m_climberSubsystem.toggleRotate();
      }
    }
  
  }
