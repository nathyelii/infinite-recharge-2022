package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase  {

    private VictorSPX climberMotor = new VictorSPX(ClimberConstants.climberChannel);
    private VictorSPX windowMotor = new VictorSPX(ClimberConstants.WINDOWMOTORCANBUSNUMBER);

    public Climber (){
        super(); 
        climberMotor.setInverted(true);
    }

    public void set(double value){
        climberMotor.set(VictorSPXControlMode.PercentOutput,value);
    }

    public void setBoth(double value){
        windowMotor.set(VictorSPXControlMode.PercentOutput,value);
        climberMotor.set(VictorSPXControlMode.PercentOutput,value);
    }
    
}
