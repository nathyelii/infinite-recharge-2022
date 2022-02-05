package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
    private VictorSPX conveyorMotor = new VictorSPX(ConveyorConstants.CoveyorCANBUSNUMBER); 
   
    public Conveyor (){
        super();
    } 

    public void set(double value){
        conveyorMotor.set(VictorSPXControlMode.PercentOutput,value);
    }
}
