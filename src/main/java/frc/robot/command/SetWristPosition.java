package frc.robot.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class SetWristPosition extends Command {
    
    private CoralSubsystem m_subsystem;
    private Rotation2d m_position;

    public SetWristPosition(CoralSubsystem subsystem, Rotation2d position) {
        this.m_subsystem = subsystem;
        this.m_position = position;

        addRequirements(subsystem); //two commands that require the same subsystem are not allowed to run at the same time
    }

    @Override
    public void initialize() {
        m_subsystem.setWristPosition(m_position);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
