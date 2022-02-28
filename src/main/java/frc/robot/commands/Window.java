package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Window  extends CommandBase {
    private final Climber m_climber;
    private double speed;
    

    public Window (Climber climber, double speed){
        super();
        m_climber = climber;
        addRequirements(climber);
        this.speed = speed;

    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climber.setWindowMotor(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climber.setWindowMotor(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
