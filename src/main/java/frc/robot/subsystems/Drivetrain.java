package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

    private final MotorController m_leftMotor;
    private final MotorController m_rightMotor;
  
    private final DifferentialDrive m_drive;
  
    // parent motors
    private final WPI_TalonSRX leftFather = new WPI_TalonSRX(DriveConstants.LEFTFATHER);
    private final WPI_TalonSRX rightFather = new WPI_TalonSRX(DriveConstants.RIGHTFATHER);
  
    // son motors
    private final WPI_VictorSPX leftSon = new WPI_VictorSPX(DriveConstants.LEFTSON);
    private final WPI_VictorSPX rightSon = new WPI_VictorSPX(DriveConstants.RIGHTSON);
  
    private final NeutralMode brakeMode = NeutralMode.Brake;


public Drivetrain()
{

}



    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    
}
