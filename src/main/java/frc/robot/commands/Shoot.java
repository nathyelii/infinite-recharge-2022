package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class Shoot  extends CommandBase {
    private final Shooter m_shooter;
    private double goalSpeed;
    BangBangController controller;
    

    public Shoot (Shooter shooter, double speed){
        super();
        m_shooter = shooter;
        addRequirements(m_shooter);
        SmartDashboard.putNumber("goalSpeed", goalSpeed);
        controller = new BangBangController();

    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        goalSpeed = SmartDashboard.getNumber("goalSpeed", 0);
        SmartDashboard.putString("Shooter On", "Yes");
        double speed = controller.calculate(m_shooter.getEncoderRate(), goalSpeed);
        SmartDashboard.putNumber("Shooter Speed", speed);
        m_shooter.set(speed);
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
