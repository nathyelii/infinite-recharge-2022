package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;

public class AutoShoot extends SequentialCommandGroup {

    public AutoShoot(Shooter shooter, Conveyor conveyor, Drivetrain m_robotDrive, String trajectoryJSON)
            throws IOException {
        addCommands(
                new Shoot(shooter).withTimeout(5.0),
                new ParallelCommandGroup(
                        new Shoot(shooter).withTimeout(5.0),
                        new ConveyorUp(conveyor).withTimeout(5.0)));
    }
}
