package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private final VictorSPX climberMotor = new VictorSPX(ClimberConstants.CLIMBER_CANBUS_NUMBER);
  private final VictorSPX lockerMotor = new VictorSPX(ClimberConstants.WINDOW_MOTOR_CANBUS_NUMBER);
  // private final Servo servo = new Servo(9);

  public Climber() {
    super();
  }

  public void setClimberMotor(double value) {
    climberMotor.set(VictorSPXControlMode.PercentOutput,
        value);
  }

  public void setLockerMotor(double value) {
    lockerMotor.set(VictorSPXControlMode.PercentOutput,
        value);
    // servo.set(.5);
  }

  public void setBoth(double value) {
    setClimberMotor(value);
    setLockerMotor(value);
  }

}
