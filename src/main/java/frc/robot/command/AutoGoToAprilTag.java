package frc.robot.command;

import org.pwrup.util.Vec2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTarget;

public class AutoGoToAprilTag extends Command {

    VisionSubsystem m_vision;
    SwerveSubsystem m_swerveSubsystem;

    double m_desiredX, m_desiredY, m_desiredRot;

    VisionTarget m_lastTarget;
    int timesSinceLastSight = 100;

    public AutoGoToAprilTag(
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem vision,
        double x,
        double y,
        double rot
    ) {
        m_vision = vision;
        m_swerveSubsystem = swerveSubsystem;

        m_desiredX = x;
        m_desiredY = y;
        m_desiredRot = rot;

        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting AutogoToApriltag");
        m_lastTarget = null;
        timesSinceLastSight = 100;
    }

    @Override
    public void execute() {
        VisionTarget target = m_vision.findBestTarget();

        // If the camera hasn't seen the target recently, uses the previous input's data
        if (target == null && timesSinceLastSight < 4) {
            target = m_lastTarget;
            timesSinceLastSight++;
        }

        if (target != null) {
            m_vision.updateCalculations(target, m_desiredX, m_desiredY, m_desiredRot);

            double sidewaysSpeedY = -m_vision.getSidewaysSpeedX();
            double forwardSpeedX = -m_vision.getForwardSpeedY();
            double rotationSpeed = -m_vision.getRotationSpeed();

            // System.out.println("sidewaysSpeedY: " + sidewaysSpeedY + " forwardSpeedX: " + forwardSpeedX + " rotationSpeed: " + rotationSpeed);

            m_swerveSubsystem.driveRaw(new Vec2(forwardSpeedX, sidewaysSpeedY), rotationSpeed, SwerveConstants.kAutonSpeedMultiplier);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(new Vec2(0, 0),0, SwerveConstants.kAutonSpeedMultiplier);
    }

    @Override
    public boolean isFinished() {
        return m_vision.isAtTarget();
    }
}
