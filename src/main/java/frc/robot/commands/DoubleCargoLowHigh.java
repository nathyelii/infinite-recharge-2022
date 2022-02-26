package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class DoubleCargoLowHigh extends SequentialCommandGroup {

    public DoubleCargoLowHigh(Shooter shooter, Conveyor conveyor, Drivetrain m_robotDrive) {
            addCommands(
                            new ShootAtSpeed(shooter, 50).withTimeout(5.0),
                            new ParallelCommandGroup(
                                            new Shoot(shooter).withTimeout(2.0),
                                            new ConveyorUp(conveyor).withTimeout(2.0)),
                            new ParallelCommandGroup(
                                            RobotContainer.followPath(m_robotDrive,  Robot.forward),
                                            new ConveyorUp(conveyor).withTimeout(4.0)),
                            new ConveyorDown(conveyor, 0).withTimeout(.5),
                            new ShootAtSpeed(shooter, 150).withTimeout(5.0),
                            new ParallelCommandGroup(
                                new Shoot(shooter).withTimeout(2.0),
                                new ConveyorUp(conveyor).withTimeout(2.0)),
                            new ParallelCommandGroup(
                                            RobotContainer.followPath(m_robotDrive,  Robot.forward),
                                            new ShootAtSpeed(shooter, 50).withTimeout(5.0))
                            );
    }

}
