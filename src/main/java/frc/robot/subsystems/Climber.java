package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase  {

    private VictorSP climberMotor = new VictorSP(ClimberConstants.climberChannel);

    public Climber (){
        super(); 
    }

    public void set(double value){
        climberMotor.set(value);
    }
    
}
