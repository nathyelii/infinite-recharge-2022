package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class SimpleAutoHighHigh extends SequentialCommandGroup {
        public SimpleAutoHighHigh(Shooter shooter, Conveyor conveyor, Drivetrain m_robotDrive, PhotonCamera m_camera) {
                addCommands(
                                new ParallelCommandGroup(
                                                new DriveAuto(-.75, m_robotDrive).withTimeout(.95),
                                                new Shoot(shooter, ShooterConstants.HIGH_GOAL_SPEED,false ).withTimeout( 2.3)),
                                new ParallelCommandGroup(
                                                new Shoot(shooter, ShooterConstants.HIGH_GOAL_SPEED,false ).withTimeout(1.0),
                                                new ConveyorUp(conveyor).withTimeout(1.0)),
                                new ParallelCommandGroup(
                                                new DriveAuto(-.75, m_robotDrive).withTimeout(1),
                                                new WarmUpShooter(shooter, 0.0).withTimeout(3.0),
                                                new ConveyorUp(conveyor).withTimeout(1.4)),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new WarmUpShooter(shooter,
                                                                                ShooterConstants.HIGH_GOAL_SPEED )      
                                                                                                .withTimeout(2.0),
                                                                new DriveAuto(.75, m_robotDrive).withTimeout(.75))),
                                                                //new CenterTarget(m_robotDrive, m_camera, true).withTimeout(.5),
                                new ParallelCommandGroup(
                                        new WarmUpShooter(shooter,
                                        ShooterConstants.HIGH_GOAL_SPEED )      
                                                        .withTimeout(0.5)
                                ),
                                new ParallelCommandGroup(
                                                new Shoot(shooter, ShooterConstants.HIGH_GOAL_SPEED,false ).withTimeout( 3.0),
                                                new ConveyorUp(conveyor).withTimeout(3.0)));
        }

}
