package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.Conveyor;

public class ConveyorDown extends CommandBase {
    private final Conveyor m_conveyor;
    private int speed;
    public ConveyorDown (Conveyor conveyor, int speed){
        super();
        m_conveyor = conveyor;
        this.speed=speed;
        addRequirements(m_conveyor);
    }

    @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(speed==0)
    {
      m_conveyor.setBoth(-1*ConveyorConstants.CONVEYOR_SPEED / 2.0);
    }
    else{
      m_conveyor.setBoth(-1*ConveyorConstants.CONVEYOR_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_conveyor.setBoth(ConveyorConstants.CONVEYOR_STOP_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
