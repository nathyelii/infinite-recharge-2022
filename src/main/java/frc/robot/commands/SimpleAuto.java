package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class SimpleAuto extends SequentialCommandGroup {
    public SimpleAuto(Shooter shooter, Conveyor conveyor, Drivetrain m_robotDrive) {
        addCommands(
                new WarmUpShooter(shooter, 50).withTimeout(5.0),
                new ParallelCommandGroup(
                        new Shoot(shooter, ShooterConstants.LOWGOALSPEED).withTimeout(2.0),
                        new ConveyorUp(conveyor).withTimeout(2.0)),
                new ParallelCommandGroup(
                        new DriveAuto(.25, m_robotDrive).withTimeout(3.0),
                        new ConveyorUp(conveyor).withTimeout(3.0)),
                new ConveyorDown(conveyor, 0).withTimeout(.5),
                new WarmUpShooter(shooter, ShooterConstants.HIGHGOALSPEED).withTimeout(5.0),
                new ParallelCommandGroup(
                        new Shoot(shooter, ShooterConstants.HIGHGOALSPEED).withTimeout(2.0),
                        new ConveyorUp(conveyor).withTimeout(2.0)),
                new ParallelCommandGroup(
                    new DriveAuto(-.25, m_robotDrive).withTimeout(3.0),
                        new WarmUpShooter(shooter, 50).withTimeout(5.0)));
    }

}
