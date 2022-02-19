package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase  {

    private VictorSPX shooterMotor = new VictorSPX(ShooterConstants.SHOOTERCANBUSNUMBER);

    public Shooter (){
        super();
    }

    public void set(double value){
        System.out.println("TESTING FROM INSIDE SHOOT SET");
        shooterMotor.set(VictorSPXControlMode.PercentOutput,value); 
    }
}
