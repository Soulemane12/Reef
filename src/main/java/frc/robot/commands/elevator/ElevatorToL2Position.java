package frc.robot.commands.elevator;

public class ElevatorToL2Position extends ElevatorPositionCommandBase {
    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(1.0));
    }
}