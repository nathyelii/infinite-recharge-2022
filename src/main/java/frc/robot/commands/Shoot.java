package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

import java.util.Arrays;

public
class Shoot extends CommandBase {
  private final Shooter m_shooter;
  BangBangController controller;
  int index = 0;
  double[] history;
  int rollingAverageSize;
  private double goalSpeed;
  private boolean readDashboard;


  public
  Shoot(Shooter shooter, double goalSpeed, boolean readDashboard) {
    super();
    m_shooter = shooter;
    addRequirements(m_shooter);
    this.readDashboard=readDashboard;
    
    this.goalSpeed = goalSpeed;
    controller = new BangBangController();
    rollingAverageSize = 5;
    SmartDashboard.putNumber("goalSpeed",
    goalSpeed);
    
  }

  @Override
  public
  void initialize() {
    if(readDashboard)
    {
      System.out.println("READING FROM DASHBOARD");
      goalSpeed = SmartDashboard.getNumber("IMUGoal",0);
    }
    SmartDashboard.delete("CanShoot");
    SmartDashboard.putNumber("goalSpeed",
                             goalSpeed);
    history = new double[rollingAverageSize];
    index = 0;
    SmartDashboard.putString("CanShoot",
                             "DON'T SHOOT");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public
  void execute() {
    goalSpeed = SmartDashboard.getNumber("goalSpeed",
                                         0);
    SmartDashboard.putString("Shooter On",
                             "Yes");
    double speed = controller.calculate(m_shooter.getEncoderRate(),
                                        goalSpeed);
    SmartDashboard.putNumber("Shooter Speed",
                             m_shooter.getEncoderRate());
    m_shooter.set(speed);
    history[++index % rollingAverageSize] = m_shooter.getEncoderRate();
    double total = Arrays.stream(history).sum();
    double average = total / rollingAverageSize;
    if (Math.abs(average - goalSpeed) > 5) {
      SmartDashboard.putString("CanShoot",
                               "SHOOT");
    } else {
      SmartDashboard.delete("CanShoot");
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public
  void end(boolean interrupted) {
    m_shooter.set(ShooterConstants.SHOOTER_SPEED_STOP);
    SmartDashboard.putString("Shooter On",
                             "No");
    SmartDashboard.delete("CanShoot");
    SmartDashboard.putString("CanShoot",
                             "DON'T SHOOT");

    m_shooter.set(0);
  }

  // Returns true when the command should end.
  @Override
  public
  boolean isFinished() {
    return false;
  }
}
