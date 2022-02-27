package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WarmUpShooter extends CommandBase {
    private final Shooter m_shooter;
    private double goalSpeed;
    BangBangController controller;
    int index =0;
    double[] history;
    int rollingAverageSize;

    public WarmUpShooter (Shooter shooter, double goalSpeed){
        super();
        controller = new BangBangController();
        SmartDashboard.putNumber("goalSpeed", goalSpeed);
        m_shooter = shooter;
        this.goalSpeed = goalSpeed;
        addRequirements(m_shooter);
        rollingAverageSize = 5;

    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("goalSpeed", goalSpeed);
        history = new double[rollingAverageSize];
        index=0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        goalSpeed = SmartDashboard.getNumber("goalSpeed", 0);
        SmartDashboard.putString("Shooter On", "Yes");
        history[++index%rollingAverageSize] = m_shooter.getEncoderRate();
        double speed = controller.calculate(m_shooter.getEncoderRate(), goalSpeed);
        SmartDashboard.putNumber("Shooter Speed", speed);
        m_shooter.set(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // double total = Arrays.stream(history).sum();
        // double average = total/rollingAverageSize;
        // if (average - goalSpeed > 0)
        // {
        //     return true;
        // } 
        return false;
    }
}

