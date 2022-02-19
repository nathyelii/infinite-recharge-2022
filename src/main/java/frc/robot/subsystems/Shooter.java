package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase  {

    private VictorSP shooterMotor = new VictorSP(ShooterConstants.SHOOTERCANBUSNUMBER);

    public Shooter (){
        super();
    }

    public void set(double value){
        shooterMotor.set(value);
    }
}
