package frc.robot.commands.elevator;

public class ElevatorToL4Position extends ElevatorPositionCommandBase {
    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(4.5));
    }
}