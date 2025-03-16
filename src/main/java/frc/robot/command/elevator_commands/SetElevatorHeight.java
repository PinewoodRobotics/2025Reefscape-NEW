package frc.robot.command.elevator_commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHeight extends Command {
    
    private ElevatorSubsystem m_elevatorSubsystem;
    private Distance m_position;
    private boolean m_waitUntilReach;

    /**
     * 
     * @param elevatorSubsystem 
     * @param height Height of the elevator, measured from top of the bottom bar of the carriage
     * @param waitUntilReach If true, waits until the elevator reaches the setpoint to end the command
     */
    public SetElevatorHeight(ElevatorSubsystem elevatorSubsystem, Distance height, boolean waitUntilReach) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_position = height;
        m_waitUntilReach = waitUntilReach;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setHeight(m_position);
        System.out.println("Starting setelevator");
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.atTarget() || !m_waitUntilReach;
    }
}

