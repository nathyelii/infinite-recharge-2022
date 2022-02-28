package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
    private VictorSPX intakeConveyorMotor = new VictorSPX(ConveyorConstants.INTAKE_CONVEYOR_CANBUS_NUMBER);
    private VictorSPX conveyorMotor = new VictorSPX(ConveyorConstants.CONVEYOR_CANBUS_NUMBER);

    public Conveyor (){
        super();
    }

    public void setBoth(double value){
        intakeConveyorMotor.set(VictorSPXControlMode.PercentOutput,value);
        conveyorMotor.set(VictorSPXControlMode.PercentOutput,value);
    }

    public void setBothOpposite(double value){
        intakeConveyorMotor.set(VictorSPXControlMode.PercentOutput,value);
        conveyorMotor.set(VictorSPXControlMode.PercentOutput,-.7*value);
    }


}
