// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Collect;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  final Drivetrain m_robotDrive = new Drivetrain();
  final Collector m_collector = new Collector();
  
  // The robot's subsystems and commands are defined here...
  private final Joystick driverLeftStick = new Joystick(0);
  private final Joystick driverRightStick = new Joystick(1);
  private final Joystick copilot = new Joystick (2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_robotDrive.setDefaultCommand(new Drive(() -> driverLeftStick.getRawAxis(0),
                () -> driverRightStick.getRawAxis(1), m_robotDrive));

                
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
    final JoystickButton collectorIn = new JoystickButton(driverRightStick, 1);
    final JoystickButton collectorOut = new JoystickButton(driverLeftStick, 1);

    //button actions here
    collectorIn.whileHeld(new Collect(m_collector, .9));
    collectorOut.whileHeld(new Collect(m_collector, -.9));

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