package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Pivot extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(ShooterConstants.kShooterMotorID, "ChooChooTrain");

    private double INITIAL_OFFSET = 0;
    private boolean hasInitialized = false;
    private double lastTargetPosition = 0;

    public Pivot() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure feedback and gear ratio if needed
        config.Feedback.SensorToMechanismRatio = 3.0; // Adjust based on gear ratio
        
        // Configure Motion Magic parameters
        config.MotionMagic.withMotionMagicCruiseVelocity(4.0)  // Reduced for more controlled movement
                          .withMotionMagicAcceleration(8.0)    // Reduced for more controlled movement
                          .withMotionMagicJerk(50.0);         // Reduced for smoother motion

        // Configure PID values
        config.Slot0.kP = 25.0;  // Increased for better position holding
        config.Slot0.kI = 0.1;   // Added small I term to eliminate steady-state error
        config.Slot0.kD = 0.2;   // Increased for better damping
        config.Slot0.kS = 0.3;   // Increased static friction compensation
        config.Slot0.kV = 1.2;   // Increased velocity feedforward
        config.Slot0.kA = 0.1;   // Increased acceleration feedforward
        config.Slot0.kG = 0.15;  // Increased gravity compensation

        // Apply configuration
        m_motor.getConfigurator().apply(config);

        // Set neutral mode
        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Moves the shooter motor to a specific position using Motion Magic.
     * @param targetPosition The target position in rotations.
     */
    public void setShooterPosition(double targetPosition) {
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motor.setControl(new MotionMagicVoltage(targetPosition));
        lastTargetPosition = targetPosition;
    }

    /**
     * Holds the last known position to prevent drifting.
     */
    public void holdPosition() {
        setShooterPosition(lastTargetPosition);
    }

    /**
     * Gets the current position of the shooter pivot.
     * @return The current position in rotations.
     */
    public double getCurrentPosition() {
        return m_motor.getPosition().getValueAsDouble() - INITIAL_OFFSET;
    }

    @Override
    public void periodic() {
        // Initialize offset once we get a non-zero reading
        if (!hasInitialized) {
            double currentPosition = m_motor.getPosition().getValueAsDouble();
            if (currentPosition != 0) {
                INITIAL_OFFSET = currentPosition;
                hasInitialized = true;
                System.out.println("Initialized shooter offset to: " + INITIAL_OFFSET);
            }
        }

        // Print position for debugging
        double position = getCurrentPosition();
        System.out.println("Shooter Position: " + position);

        SmartDashboard.putNumber("Pivot Position", m_motor.getPosition().getValueAsDouble());
    }
}
