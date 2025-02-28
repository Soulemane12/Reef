package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorJoystickCommand extends Command {
    private final Elevator m_elevator;
    private final CommandXboxController m_joystick;
    private final double kElevatorGravityCompensation;

    public ElevatorJoystickCommand(Elevator elevator, CommandXboxController joystick, double gravityCompensation) {
        m_elevator = elevator;
        m_joystick = joystick;
        kElevatorGravityCompensation = gravityCompensation;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double rawSpeed = m_joystick.getLeftY();
        double speed = applyDeadband(rawSpeed, 0.05);

        if (speed == 0) {
            m_elevator.moveElevator(kElevatorGravityCompensation);
        } else if (speed < 0) {
            m_elevator.moveElevator(-speed + kElevatorGravityCompensation);
        } else {
            m_elevator.moveElevator(-speed);
        }
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0 : value;
    }
}