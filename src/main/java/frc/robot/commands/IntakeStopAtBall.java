package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.Conveyor;

public class IntakeStopAtBall  extends CommandBase{ 
    private final Conveyor m_conveyor; 
    public IntakeStopAtBall(Conveyor conveyor){
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
            m_conveyor.set(ConveyorConstants.conveyorSpeed);

        }
      
        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            m_conveyor.set(0); 
        }
      
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
         double color= m_conveyor.readColorSensor();
         SmartDashboard.putNumber("color", color);
         if(Math.abs(color)>ConveyorConstants.COLORTHRESHOLD)
         {
            return true; 
         }
         else{
            return false; 
         }
        }
    }
