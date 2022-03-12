package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IronMechEncoder
{
    private WPI_TalonSRX motor;
    private double distancePerPulse;
    private int dir;


    public IronMechEncoder( WPI_TalonSRX motor)
    {
        this.motor=motor;
        distancePerPulse=0;
        dir = 1;

    }

    public void setInverted(){
        dir = -1;
    }

  public
  void setDistancePerPulse(double distancePerPulse)
  {
    this.distancePerPulse = distancePerPulse;
  }

    public double getDistance()
    {
        return motor.getSelectedSensorPosition()*distancePerPulse*dir;
    }

    public double getRate(){
        return motor.getSelectedSensorVelocity()*distancePerPulse*dir;
    }

  public
  void reset() {
    motor.setSelectedSensorPosition(0);
  }

}
