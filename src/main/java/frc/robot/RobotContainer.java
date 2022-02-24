// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ConveyorColor;
import frc.robot.commands.ConveyorDown;
import frc.robot.commands.ConveyorUp;
import frc.robot.commands.Drive;
import frc.robot.commands.DriverConveyorUp;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;

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
  final Shooter m_shooter = new Shooter();
  
  // The robot's subsystems and commands are defined here...
  private final Joystick driverLeftStick = new Joystick(0);
  private final Joystick driverRightStick = new Joystick(1);
  private final Joystick copilot = new Joystick (2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_robotDrive.setDefaultCommand(new Drive(() -> driverLeftStick.getX(),
                () -> driverRightStick.getY(), m_robotDrive));
    m_robotConveyor.setDefaultCommand(new ConveyorColor(m_robotConveyor));
                
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
    final JoystickButton climbUp = new JoystickButton(driverLeftStick, 5);
    final JoystickButton climbDown = new JoystickButton(driverLeftStick, 3);
    final JoystickButton conveyorUp = new JoystickButton(copilot, 11);
      final JoystickButton driverCollect = new JoystickButton(driverRightStick, 4);
      final JoystickButton conveyorDownSlow = new JoystickButton(copilot, 10);
    final JoystickButton shoot = new JoystickButton(copilot, 1);

    //button actions here
    climbUp.whileHeld(new ClimbUp(m_robotClimber));
    climbDown.whileHeld(new ClimbDown(m_robotClimber)); 
     conveyorUp.whileHeld(new ConveyorUp(m_robotConveyor));
     driverCollect.whileHeld(new DriverConveyorUp(m_robotConveyor));
     conveyorDownSlow.whileHeld(new ConveyorDown(m_robotConveyor,0));
    shoot.toggleWhenPressed(new Shoot(m_shooter));

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
