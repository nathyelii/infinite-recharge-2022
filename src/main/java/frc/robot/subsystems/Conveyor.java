package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
    private VictorSPX intakeConveyorMotor = new VictorSPX(ConveyorConstants.intakeConveyorCANBUSNUMBER); 
    private VictorSPX conveyorMotor = new VictorSPX(ConveyorConstants.conveyorCANBUSNUMBER);
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    
    public Conveyor (){
        super();
    } 

    public void set(double value){
        intakeConveyorMotor.set(VictorSPXControlMode.PercentOutput,value);
        conveyorMotor.set(VictorSPXControlMode.PercentOutput,value); 
    }

    public double readColorSensor (){
        Color detectedColor = m_colorSensor.getColor();
        return detectedColor.red- detectedColor.blue; 
    }


    // Color detectedColor = m_colorSensor.getColor();
    // double IR = m_colorSensor.getIR();

    // SmartDashboard.putNumber("red", detectedColor.red);
    // SmartDashboard.putNumber("green", detectedColor.green);
    // SmartDashboard.putNumber("blue", detectedColor.blue);
    // SmartDashboard.putNumber("IR", IR);
}
