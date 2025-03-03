package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Shooter;

public class ElevatorToL4Position extends ElevatorPositionCommandBase {
    private static Shooter m_shooter;
    private boolean positionReached = false;
    private final Timer shootTimer = new Timer();
    private static final double SHOOT_DURATION = 1.0;

    public static void initialize(Shooter shooter) {
        m_shooter = shooter;
    }

    public ElevatorToL4Position() {
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(4.5));
        /* 
       
        // Check if we're at position (within tolerance)
        if (Math.abs(m_elevator.getPosition() - 4.5) < 0.1 && !positionReached) {
            positionReached = true;
            m_shooter.shoot(1.0);
            shootTimer.restart();
        }

        // Stop shooting after SHOOT_DURATION seconds
        if (positionReached && shootTimer.hasElapsed(SHOOT_DURATION)) {
            m_shooter.shoot(0.0);
        }
            */
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0.0);
        positionReached = false;
        shootTimer.stop();
    }
}