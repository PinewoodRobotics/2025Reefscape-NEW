package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeIntake extends Command {
    
    private AlgaeSubsystem m_subsystem;

    public AlgaeIntake(AlgaeSubsystem subsystem) {
        this.m_subsystem = subsystem;

        addRequirements(subsystem); //two commands that require the same subsystem are not allowed to run at the same time
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_subsystem.runMotors();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
