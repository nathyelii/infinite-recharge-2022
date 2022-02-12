// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ConveyorDown;
import frc.robot.commands.ConveyorUp;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeIn;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  final Drivetrain m_robotDrive = new Drivetrain();
  final Climber m_robotClimber = new Climber();
  final Conveyor m_robotConveyor = new Conveyor();
  final Intake m_intake = new Intake();
  
  // The robot's subsystems and commands are defined here...
  private final Joystick driverLeftStick = new Joystick(0);
  private final Joystick driverRightStick = new Joystick(1);
  private final Joystick copilot = new Joystick (2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_robotDrive.setDefaultCommand(new Drive(() -> driverLeftStick.getX(),
                () -> driverRightStick.getY(), m_robotDrive));

                
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  //buttons here
    final JoystickButton climbUp = new JoystickButton(driverRightStick, 2);
    final JoystickButton climbDown = new JoystickButton(driverLeftStick, 2);
    final JoystickButton intakeIn = new JoystickButton(driverRightStick, 1);
    final JoystickButton intakeOut = new JoystickButton(driverLeftStick, 3);
    final JoystickButton conveyorUp = new JoystickButton(driverRightStick, 4);
      final JoystickButton conveyorDown = new JoystickButton(driverLeftStick, 4);
      final JoystickButton driveStraight = new JoystickButton(copilot, 1);

    //button actions here
    climbUp.whileHeld(new ClimbUp(m_robotClimber));
    climbDown.whileHeld(new ClimbDown(m_robotClimber)); 
     conveyorUp.whileHeld(new ConveyorUp(m_robotConveyor));
     conveyorDown.whileHeld(new ConveyorDown(m_robotConveyor));
     intakeIn.whileHeld(new IntakeIn(m_intake));
     driveStraight.whileHeld(new DriveStraight(m_robotDrive));

  } 

    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}