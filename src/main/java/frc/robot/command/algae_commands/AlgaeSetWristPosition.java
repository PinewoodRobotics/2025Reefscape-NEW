package frc.robot.command.algae_commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class SetWristPosition extends Command {

    private AlgaeSubsystem m_algaeSubsystem;
    private Rotation2d m_position;
    private boolean m_waitUntilReach;

    public SetWristPosition(AlgaeSubsystem algaeSubsystem, Rotation2d position, boolean waitUntilReach) {
        m_algaeSubsystem = algaeSubsystem;
        m_position = position;
        m_waitUntilReach = waitUntilReach;

        addRequirements(algaeSubsystem);
    }

    @Override
    public void initialize() {
        m_algaeSubsystem.setWristPosition(m_position);
    }
    
    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return m_algaeSubsystem.atSetpoint() || !m_waitUntilReach;
    }
    
}