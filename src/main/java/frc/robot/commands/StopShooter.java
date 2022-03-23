package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StopShooter extends CommandBase{
    private final Shooter m_shooter;
  int index = 0;
  double[] history;
  int rollingAverageSize;
  private double goalSpeed;


  public
  StopShooter(Shooter shooter, double goalSpeed) {
    super();
    m_shooter = shooter;
    addRequirements(m_shooter);
    this.goalSpeed = goalSpeed;

  }

  @Override
  public
  void initialize() {
    m_shooter.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public
  void execute() {
    if(m_shooter.getEncoderRate() > goalSpeed){
        m_shooter.set(-1);
      }


  }

  // Called once the command ends or is interrupted.
  @Override
  public
  void end(boolean interrupted) {
   
    m_shooter.set(0);
  }

  // Returns true when the command should end.
  @Override
  public
  boolean isFinished() {
    return false;
  }
}
