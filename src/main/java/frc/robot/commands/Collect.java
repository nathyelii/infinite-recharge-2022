package frc.robot.commands;

import frc.robot.subsystems.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Collect extends CommandBase {
    private final Collector collector;
    private final double speed;

    public Collect(Collector collector, double speed){
        super();
        this.collector = collector;
        this.speed = speed;
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        collector.run(speed);
    }

    @Override
    public void end(boolean interrupted){
        collector.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
