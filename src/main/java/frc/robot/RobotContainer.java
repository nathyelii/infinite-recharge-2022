// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ConveyorDown;
import frc.robot.commands.ConveyorUp;
import frc.robot.commands.DoubleCargoLowHigh;
import frc.robot.commands.DoubleCargoLowLow;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.DriverConveyorUp;
import frc.robot.commands.Shoot;
import frc.robot.commands.SimpleAuto;
import frc.robot.commands.SimpleAutoHighHigh;
import frc.robot.commands.SimpleAutoLowLow;
import frc.robot.commands.Window;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public
class RobotContainer {

  final Drivetrain m_drivetrain = new Drivetrain();
  final Climber m_climber = new Climber();
  final Conveyor m_conveyor = new Conveyor();
  final Shooter m_shooter = new Shooter();

  // The robot's subsystems and commands are defined here...
  private final Joystick driverLeftStick = new Joystick(0);
  private final Joystick driverRightStick = new Joystick(1);
  private final Joystick copilot = new Joystick(2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public
  RobotContainer() {

    m_drivetrain.setDefaultCommand(new Drive(driverLeftStick::getX,
                                             driverRightStick::getY,
                                             m_drivetrain));

    // Configure the button bindings
    configureButtonBindings();
  }

  public static
  RamseteCommand followPath(Drivetrain m_robotDrive, Trajectory path) {
    System.out.println("In follow Path");
    return new RamseteCommand(path,
                              m_robotDrive::getPose,
                              new RamseteController(AutoConstants.kRamseteB,
                                                    AutoConstants.kRamseteZeta),
                              new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                                         DriveConstants.kvVoltSecondsPerMeter,
                                                         DriveConstants.kaVoltSecondsSquaredPerMeter),
                              DriveConstants.kDriveKinematics,
                              m_robotDrive::getWheelSpeeds,
                              new PIDController(DriveConstants.kPDriveVel,
                                                0,
                                                0),
                              new PIDController(DriveConstants.kPDriveVel,
                                                0,
                                                0),
                              // RamseteCommand passes volts to the callback
                              m_robotDrive::tankDriveVolts,
                              m_robotDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private
  void configureButtonBindings() {

    // buttons here
    final JoystickButton climbUp = new JoystickButton(driverLeftStick, 5);
    final JoystickButton climbDown = new JoystickButton(driverLeftStick, 3);
    final JoystickButton windowUp = new JoystickButton(driverRightStick, 5);
    final JoystickButton windowDown = new JoystickButton(driverRightStick, 3);
    final JoystickButton conveyorUp = new JoystickButton(copilot, 6);
    final JoystickButton conveyorUpTrigger = new JoystickButton(copilot, 1);
    final JoystickButton driverCollect = new JoystickButton(driverLeftStick, 4);
    final JoystickButton conveyorDownSlow = new JoystickButton(copilot, 7);
    final JoystickButton shootHigh = new JoystickButton(copilot, 3);
    final JoystickButton shootLow = new JoystickButton(copilot, 2);
    final JoystickButton reverseLockerButton = new JoystickButton(driverRightStick, 3);

    // button actions here
    climbUp.whileHeld(new ClimbUp(m_climber));
    climbDown.whileHeld(new ClimbDown(m_climber));
    conveyorUp.whileHeld(new ConveyorUp(m_conveyor));
    driverCollect.whileHeld(new DriverConveyorUp(m_conveyor));
    conveyorDownSlow.whileHeld(new ConveyorDown(m_conveyor,
                                                0));
    shootHigh.whileHeld(new Shoot(m_shooter,
                                  ShooterConstants.HIGH_GOAL_SPEED));
    shootLow.whileHeld(new Shoot(m_shooter,
                                 ShooterConstants.LOW_GOAL_SPEED));
    conveyorUpTrigger.whileHeld(new ConveyorUp(m_conveyor));
    windowUp.whileHeld(new Window(m_climber,
                                  .25));
    windowDown.whileHeld(new Window(m_climber,
                                    -.25));
    // reverseLockerButton.whileHeld();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String autoMode,boolean override) {

    if(override)
    {
      SmartDashboard.putString("AutoMode", "SIMPLEAUTO");
        return new SimpleAuto(m_shooter, m_robotConveyor, m_robotDrive);
        // return new SimpleAutoHighHigh(m_shooter, m_robotConveyor, m_robotDrive);
        // SmartDashboard.putString("AutoMode", "SIMPLEAUTOHIGHHIGH");
    }
    SmartDashboard.putString("AutoMode", "NONE");
    switch (autoMode) {
      case AutoConstants.SIMPLE_AUTO:
        SmartDashboard.putString("AutoMode",
                                 "SIMPLEAUTO");
        return new SimpleAuto(m_shooter,
                              m_conveyor,
                              m_drivetrain);
      case AutoConstants.DRIVE:
        SmartDashboard.putString("AutoMode",
                                 "DRIVE");
        return new DriveAuto(1,
                             m_drivetrain).withTimeout(3.0);
      case AutoConstants.DOUBLE_CARGO_LOW_LOW:
        SmartDashboard.putString("AutoMode",
                                 "DOUBLECARGOLOWLOW");
        return new DoubleCargoLowLow(m_shooter,
                                     m_conveyor,
                                     m_drivetrain);
      case AutoConstants.DOUBLE_CARGO_LOW_HIGH:
        SmartDashboard.putString("AutoMode",
                                 "DOUBLECARGOLOWHIGH");
        return new DoubleCargoLowHigh(m_shooter,
                                      m_conveyor,
                                      m_drivetrain);
      case AutoConstants.SIMPLE_AUTO_LOW_LOW:
        SmartDashboard.putString("AutoMode",
                                 "SIMPLEAUTOLOWLOW");
        return new SimpleAutoLowLow(m_shooter,
                                    m_conveyor,
                                    m_drivetrain);
      case AutoConstants.SIMPLE_AUTO_HIGH_HIGH:
        SmartDashboard.putString("AutoMode",
                                 "SIMPLEAUTOHIGHHIGH");
        return new SimpleAutoHighHigh(m_shooter,
                                      m_conveyor,
                                      m_drivetrain);
    }

    return null;
  }
}
