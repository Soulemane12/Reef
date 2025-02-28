package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class ElevatorToL4Position extends Command {
    private final Elevator m_elevator;
    private final MotionMagicVoltage m_request;

    public ElevatorToL4Position(Elevator elevator, MotionMagicVoltage request) {
        m_elevator = elevator;
        m_request = request;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(4.5));
    }
}