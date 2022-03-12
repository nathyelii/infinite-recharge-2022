package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class SimpleAutoHighHigh extends SequentialCommandGroup {
        public SimpleAutoHighHigh(Shooter shooter, Conveyor conveyor, Drivetrain m_robotDrive) {
                addCommands(
                                new ParallelCommandGroup(
                                                new DriveAuto(-.75, m_robotDrive).withTimeout(.25),
                                                new Shoot(shooter, ShooterConstants.HIGHGOALSPEED).withTimeout(2.0)),
                                new ParallelCommandGroup(
                                                new Shoot(shooter, ShooterConstants.HIGHGOALSPEED).withTimeout(2.5),
                                                new ConveyorUp(conveyor).withTimeout(1.0)),
                                new ParallelCommandGroup(
                                                new DriveAuto(-.75, m_robotDrive).withTimeout(1),
                                                new WarmUpShooter(shooter, 0.0).withTimeout(3.0),
                                                new ConveyorUp(conveyor).withTimeout(2.5)),
                                new ConveyorDown(conveyor, 0).withTimeout(.25),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new WarmUpShooter(shooter,
                                                                                ShooterConstants.HIGHGOALSPEED)
                                                                                                .withTimeout(2.0),
                                                                new DriveAuto(.75, m_robotDrive).withTimeout(.75))),
                                new ParallelCommandGroup(
                                                new Shoot(shooter, ShooterConstants.HIGHGOALSPEED).withTimeout(3.0),
                                                new ConveyorUp(conveyor).withTimeout(3.0)));
        }

}