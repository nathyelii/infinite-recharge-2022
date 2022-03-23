package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public
class DriveAuto extends CommandBase {


  private final Drivetrain m_drivetrain;
  private final double speed;


  /**
   * Creates a new TankDrive command.
   *
   * @param left       The control input for the left side of the drive
   * @param right      The control input for the right sight of the drive
   * @param drivetrain The drivetrain subsystem to drive
   */
  public
  DriveAuto(double speed, Drivetrain drivetrain) {
    super();
    m_drivetrain = drivetrain;
    this.speed = speed;
    addRequirements(m_drivetrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public
  void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public
  void execute() {
    m_drivetrain.arcadeDrive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public
  void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public
  boolean isFinished() {
    return false;
  }

}
