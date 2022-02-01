package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlopperConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Collector extends SubsystemBase {
    private final WPI_VictorSPX collectorMotor;

    public Collector () {
        super();
        collectorMotor = new WPI_VictorSPX(FlopperConstants.collectorMotor);
    }
    
    public void run (double speed){
        collectorMotor.set(speed);
    }

    public void stop (){
        collectorMotor.set(0);
    }
}
