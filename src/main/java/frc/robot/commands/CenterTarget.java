package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CenterTarget extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final PhotonCamera m_camera;
    final double ANGULAR_P = 0.05;
    final double ANGULAR_D = 0;
    PIDController turnController;
    double rotationSpeed;
    PhotonPipelineResult result;
    private final double TARGET_HEIGHT_METERS  = Units.feetToMeters(8) + Units.inchesToMeters(8);
    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(29);
    private final double CAMERA_PITCH_RADRINAS = Units.degreesToRadians(23);
    private boolean move;

    public CenterTarget(Drivetrain drivetrain, PhotonCamera camera, boolean move) {
        super();
        m_drivetrain = drivetrain;
        m_camera = camera;
        this.move = move;
        turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        rotationSpeed = 0;
        m_camera.takeInputSnapshot();
        m_camera.takeOutputSnapshot();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        result = m_camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();

            double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADRINAS, Units.degreesToRadians(bestTarget.getPitch()));

            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            double angle = bestTarget.getYaw();
            SmartDashboard.putNumber("Angle", angle);
            SmartDashboard.putNumber("Range", Units.metersToInches(range));
            rotationSpeed = -turnController.calculate(angle, 0);
            
        } else {
            SmartDashboard.putNumber("Angle", -7);
            SmartDashboard.putNumber("Range", -7);
            // If we have no targets, stay still.
            rotationSpeed = 0;
            SmartDashboard.putNumber("rotationSpeed", rotationSpeed);
        }
        if(move)
        {
        m_drivetrain.arcadeDrive(0, rotationSpeed);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_camera.takeInputSnapshot();
        m_camera.takeOutputSnapshot();
        m_drivetrain.tankDriveVolts(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (result.hasTargets() &&  result.getBestTarget().getYaw() < Math.toRadians(10)) {
            return true;
        }
        return false;
    }

}
