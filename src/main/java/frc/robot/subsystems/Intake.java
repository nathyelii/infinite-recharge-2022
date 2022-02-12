package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private VictorSPX conveyorMotor = new VictorSPX(IntakeConstants.intakeCANBUSNUMBER); 
   
    public Intake (){
        super();
    } 

    public void set(double value){
        conveyorMotor.set(VictorSPXControlMode.PercentOutput,value);
    }
}
