package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FollowPath extends CommandBase {


    private final Drivetrain m_drivetrain;
  
  
    /**
     * Creates a new TankDrive command.
     *
     * @param drivetrain The drivetrain subsystem to drive
     */
    public FollowPath(Drivetrain drivetrain) {
      super();
      m_drivetrain = drivetrain;
      addRequirements(m_drivetrain);
    }
  
  
  // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_drivetrain.log();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_drivetrain.arcadeDrive(0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
