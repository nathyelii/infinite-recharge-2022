package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimbDown extends CommandBase {
    private final Climber m_climber; 

    public ClimbDown (Climber climber){
        super(); 
        m_climber = climber; 
        addRequirements(m_climber);
    }
    
        @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_climber.setBoth(-1*ClimberConstants.CLIMBSPEEDDOWN);
      m_climber.setWindowMotor(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_climber.setBoth(ClimberConstants.climbSpeedStop); 
       
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
