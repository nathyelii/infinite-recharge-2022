package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public
class Shooter extends SubsystemBase {

  private VictorSPX shooterMotor = new VictorSPX(ShooterConstants.SHOOTER_CANBUS_NUMBER);
  private Encoder encoder;

  public
  Shooter() {
    super();
    encoder = new Encoder(1,
                          2,
                          false,
                          Encoder.EncodingType.k4X);
    encoder.setDistancePerPulse(1.0 / 2048);
    shooterMotor.setInverted(true);

    // Configures the encoder to consider itself stopped after .1 seconds
    encoder.setMaxPeriod(.1);

    // Configures the encoder to consider itself stopped when its rate is below 10
    encoder.setMinRate(10);

    // Reverses the direction of the encoder
    encoder.setReverseDirection(true);

    // Configures an encoder to average its period measurement over 5 samples
    // Can be between 1 and 127 samples
    encoder.setSamplesToAverage(5);
  }

  public
  double getEncoderRate()
  {
    return encoder.getRate();
  }

  public
  void set(double value) {
    shooterMotor.set(VictorSPXControlMode.PercentOutput,
                     value);
  }

  public
  void bangBangShoot(double value) {
    //TODO set up encoder
  }
}
