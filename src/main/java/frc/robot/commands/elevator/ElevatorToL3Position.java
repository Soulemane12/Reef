package frc.robot.commands.elevator;

public class ElevatorToL3Position extends ElevatorPositionCommandBase {
    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(3.0));
    }
}