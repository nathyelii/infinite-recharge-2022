package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IronMechEncoder;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

    private final MotorControllerGroup m_leftMotor;
    private final MotorControllerGroup m_rightMotor;

    private final DifferentialDrive m_drive;

    // parent motors
    private final WPI_TalonSRX leftFather = new WPI_TalonSRX(DriveConstants.LEFTFATHER);
    private final WPI_TalonSRX rightFather = new WPI_TalonSRX(DriveConstants.RIGHTFATHER);

    // son motors
    private final WPI_VictorSPX leftSon = new WPI_VictorSPX(DriveConstants.LEFTSON);
    private final WPI_VictorSPX rightSon = new WPI_VictorSPX(DriveConstants.RIGHTSON);

    private final NeutralMode brakeMode = NeutralMode.Brake;

    private final IronMechEncoder leftEncoder;
    private final IronMechEncoder rightEncoder;

    private DifferentialDriveOdometry m_odometry;

    private boolean isForward;

    public static ADIS16448_IMU imu = new ADIS16448_IMU();

    public Drivetrain() {
        super();

        imu.calibrate();

        imu.setYawAxis(IMUAxis.kY);

        isForward = true;

        leftFather.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rightFather.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        leftFather.configFactoryDefault();
        rightFather.configFactoryDefault();

        rightFather.setNeutralMode(brakeMode);
        leftFather.setNeutralMode(brakeMode);
        rightSon.setNeutralMode(brakeMode);
        leftSon.setNeutralMode(brakeMode);
        leftFather.setSensorPhase(true);
        rightFather.setSensorPhase(true);
        leftFather.setInverted(false);
        rightFather.setInverted(false);

        leftSon.follow(leftFather);
        rightSon.follow(rightFather);

        setTalon(leftFather);
        setTalon(rightFather);

        rightFather.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
        rightFather.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

        leftFather.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
        leftFather.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

        // Let's name the sensors on the LiveWindow

        m_leftMotor = new MotorControllerGroup(leftFather, leftSon);
        m_rightMotor = new MotorControllerGroup(rightFather, rightSon);
        m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        m_drive.setSafetyEnabled(false);
        addChild("Drive", m_drive);

        leftEncoder = new IronMechEncoder(leftFather);
        rightEncoder = new IronMechEncoder(rightFather);

        leftEncoder.setDistancePerPulse(DriveConstants.METERSPERPULSE);
        rightEncoder.setDistancePerPulse(DriveConstants.METERSPERPULSE);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(imu.getGyroAngleY()));
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        // System.out.println("Left: " + leftVolts +"\tRight:" + rightVolts);
        leftFather.setVoltage(leftVolts);
        rightFather.setVoltage(rightVolts);
    }

    /**
     * Tank style driving for the DriveTrain.
     *
     * @param left  Speed in range [-1,1]
     * @param right Speed in range [-1,1]
     */
    public void drive(final double left, final double right) {
        if (isForward) {
            m_drive.tankDrive(left, right);
        } else {
            m_drive.tankDrive(-right, -left);

        }
    }

    public double getAverageEncoderDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
      }
    
      public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
      }
    
      public void zeroHeading() {
        imu.reset();
      }
    
      public double getHeading() {
        return imu.getGyroAngleY();
      }
    
      public double getTurnRate() {
        return -imu.getRate();
      }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public Object arcadeDrive(double fwd, double rot) {
        // rightFather.set(ControlMode.PercentOutput,fwd+rot);
        // leftFather.set(ControlMode.PercentOutput,fwd-rot);
        if (Math.abs(fwd) <= .05) {
          fwd = 0;
        }
        if (Math.abs(rot) <= .05) {
          rot = 0;
        }
        m_drive.arcadeDrive(fwd, -1 * rot,true);
        // m_drive.tankDrive(left, right);
        return null;
      }

    public void setTalon(final WPI_TalonSRX _talon) {

        /* Set relevant frame periods to be at least as fast as periodic rate */
        _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

        /* Set the peak and nominal outputs */
        _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
        _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
        _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */
        _talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        _talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        _talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        _talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        _talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

        /* Set acceleration and vcruise velocity - see documentation */
        _talon.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
        _talon.configMotionAcceleration(6000, Constants.kTimeoutMs);

        /* Zero the sensor once on robot boot up */
        _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

}
