package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class SimpleAuto extends SequentialCommandGroup {
        public SimpleAuto(Shooter shooter, Conveyor conveyor, Drivetrain m_robotDrive) {
                addCommands(
                                new Shoot(shooter, ShooterConstants.LOW_GOAL_AUTO_SPEED ).withTimeout( 2.0),
                                new ParallelCommandGroup(
                                                new Shoot(shooter, ShooterConstants.LOW_GOAL_AUTO_SPEED ).withTimeout( 1.0),
                                                new ConveyorUp(conveyor).withTimeout(1.0)),
                                new ParallelCommandGroup(
                                                new Shoot(shooter, -1 * ShooterConstants.HIGH_GOAL_SPEED ).withTimeout( 2.5),
                                                //this was 2.0 before playoffs
                                                //issue with auto first time after Hatsboro need to fix.
                                                //ball keeps "pooping"
                                                new DriveAuto(-.75, m_robotDrive).withTimeout(2.0-.4),
                                                new ConveyorUp(conveyor).withTimeout(3.1/2)),
                                new ConveyorUp(conveyor).withTimeout(.75),
                                new ParallelCommandGroup(
                                                new ConveyorDown(conveyor, 0).withTimeout(.5),

                                                new DriveAuto(.75, m_robotDrive).withTimeout(1.18-.4),
                                                new WarmUpShooter(shooter, ShooterConstants.HIGH_GOAL_SPEED )
                                                                .withTimeout(2.0)),
                                new ParallelCommandGroup(
                                                new Shoot(shooter, ShooterConstants.HIGH_GOAL_SPEED ).withTimeout( 3.0),
                                                new ConveyorUp(conveyor).withTimeout(3.0)));
        }

}
