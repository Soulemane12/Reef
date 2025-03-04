package frc.robot.commands.elevator;

import frc.robot.Constants;

public class ElevatorToL2Position extends ElevatorPositionCommandBase {
    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(Constants.ElevatorConstants.kL2Position));
    }
}