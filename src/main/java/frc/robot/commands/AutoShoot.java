package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoShoot extends CommandBase {

    private final Shooter m_shooter;
    private final PhotonCamera m_camera;
    PhotonPipelineResult result;

     public AutoShoot(Shooter shooter, PhotonCamera camera) {
        super();
        m_shooter = shooter;
        m_camera = camera;
        addRequirements(shooter);
    }

    private double equation(double input)
    {
        return .25*input+56.4;
    }

    @Override
    public void initialize() {
        m_camera.takeInputSnapshot();
        m_camera.takeOutputSnapshot();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        result = m_camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();

            double range = PhotonUtils.calculateDistanceToTargetMeters(CameraConstants.CAMERA_HEIGHT_METERS, CameraConstants.TARGET_HEIGHT_METERS,
            CameraConstants.CAMERA_PITCH_RADRINAS, Units.degreesToRadians(bestTarget.getPitch()));
                    //the equation we used is for inches
                    m_shooter.set(equation(Units.metersToInches(range)));
            
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_camera.takeInputSnapshot();
        m_camera.takeOutputSnapshot();
        m_shooter.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
