package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class Shoot  extends CommandBase {
    private final Shooter m_shooter;

    public Shoot (Shooter shooter){
        super();
        m_shooter = shooter;
        addRequirements(m_shooter);


    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double value = SmartDashboard.getNumber("shootingSpeed", .50);
        SmartDashboard.putString("Shooter On", "Yes");
        SmartDashboard.putNumber("Shooter Speed", m_shooter.getEncoderRate());
        m_shooter.set(-1*value);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.set(ShooterConstants.SHOOTERSPEEDSTOP);
        SmartDashboard.putString("Shooter On", "No");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
