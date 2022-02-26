package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShootAtSpeed extends CommandBase {
    private final Shooter m_shooter;
    private double goalSpeed;
    private double speed = 0.01;
    BangBangController controller;

    public ShootAtSpeed (Shooter shooter, double goalSpeed){
        super();
        controller = new BangBangController();
        m_shooter = shooter;
        this.goalSpeed = goalSpeed;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("goalSpeed", goalSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        goalSpeed = SmartDashboard.getNumber("goalSpeed", 0);
        SmartDashboard.putNumber("speedSpeed", speed);
        SmartDashboard.putString("Shooter On", "Yes");
        SmartDashboard.putNumber("Shooter Speed", m_shooter.getEncoderRate());
        speed = controller.calculate(m_shooter.getEncoderRate(), goalSpeed);
        m_shooter.set(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if(m_shooter.getEncoderRate()>=goalSpeed)
        // {
        //     return true;
        // }
        return false;
    }
}

