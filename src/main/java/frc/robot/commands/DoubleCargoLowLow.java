package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class DoubleCargoLowLow extends SequentialCommandGroup {

        public DoubleCargoLowLow(Shooter shooter, Conveyor conveyor, Drivetrain m_robotDrive) {
                addCommands(
                                new WarmUpShooter(shooter, 10).withTimeout(5.0),
                                new ParallelCommandGroup(
                                                new Shoot(shooter, ShooterConstants.LOW_GOAL_SPEED).withTimeout(2.0),
                                                new ConveyorUp(conveyor).withTimeout(2.0)),
                                RobotContainer.followPath(m_robotDrive, Robot.forward).deadlineWith(
                                                new ConveyorUp(conveyor)),
                                new ConveyorDown(conveyor, 0).withTimeout(.5),
                                RobotContainer.followPath(m_robotDrive, Robot.backward)
                                                .deadlineWith(new WarmUpShooter(shooter, 50)),
                                new ParallelCommandGroup(
                                                new Shoot(shooter, ShooterConstants.LOW_GOAL_SPEED).withTimeout(2.0),
                                                new ConveyorUp(conveyor).withTimeout(2.0)));
        }

}
