package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase  {

    private VictorSPX climberMotor = new VictorSPX(ClimberConstants.climberChannel);

    public Climber (){
        super(); 
    }

    public void set(double value){
        climberMotor.set(VictorSPXControlMode.PercentOutput,value);
    }
    
}
