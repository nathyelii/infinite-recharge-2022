package frc.robot.commands;

import java.util.ArrayList;
import java.util.Stack;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.Conveyor;

public class ConveyorColor extends CommandBase{
    
    private final Conveyor m_conveyor; 
    private ArrayList<Integer> list = new ArrayList<>();
    public ConveyorColor (Conveyor conveyor){
        super();
        m_conveyor = conveyor; 
        addRequirements(m_conveyor);
    }

    @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double color= m_conveyor.readColorSensor();
    Integer distance = m_conveyor.readDistanceSensor();
    list.add(distance);
    if(list.size()>15)
    {
        list.remove(0);
    }
    int sum = 0;
    for(Integer num: list)
    {
        sum+= num;
    }
    SmartDashboard.putNumber("color", color);
    SmartDashboard.putNumber("distance",sum/list.size());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_conveyor.setBoth(ConveyorConstants.conveyorSpeedStop); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}