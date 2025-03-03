package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(PivotConstants.kPivotMotorID, "ChooChooTrain");

    private double INITIAL_OFFSET = 0;
    private boolean hasInitialized = false;
    private double lastTargetPosition = 0;

    public Pivot() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure feedback and gear ratio if needed
        config.Feedback.SensorToMechanismRatio = 3.0; // Adjust based on gear ratio
        
        // Configure Motion Magic parameters - slower but more precise movement
        config.MotionMagic.withMotionMagicCruiseVelocity(3.0)  // Slower for more precision
                          .withMotionMagicAcceleration(6.0)    // Slower acceleration
                          .withMotionMagicJerk(30.0);         // Reduced jerk for smoother stops

        // Configure PID values - more aggressive position holding
        config.Slot0.kP = 35.0;  // More aggressive position holding
        config.Slot0.kI = 0.05;  // Reduced I to prevent wind-up
        config.Slot0.kD = 0.8;   // Increased D for better deceleration
        config.Slot0.kS = 0.35;  // Slightly increased static friction compensation
        config.Slot0.kV = 1.5;   // Increased velocity feedforward
        config.Slot0.kA = 0.15;  // Increased acceleration feedforward
        config.Slot0.kG = 0.25;  // Increased gravity compensation significantly

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
        // Add a small offset to compensate for consistent undershoot
        double adjustedTarget = targetPosition * 1.05; // Scale target by 5% to compensate for undershoot
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motor.setControl(new MotionMagicVoltage(adjustedTarget));
        lastTargetPosition = adjustedTarget;
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

        // Update SmartDashboard
        SmartDashboard.putNumber("Pivot/Current Position", position);
        SmartDashboard.putNumber("Pivot/Target Position", lastTargetPosition);
        SmartDashboard.putBoolean("Pivot/At Target", Math.abs(position - lastTargetPosition) < PivotConstants.kPositionTolerance);
        SmartDashboard.putNumber("Pivot/Position Error", lastTargetPosition - position);
        
        // Add named positions for reference
        SmartDashboard.putNumber("Pivot/L0 Position", PivotConstants.kL0Position);
        SmartDashboard.putNumber("Pivot/L2 Position", PivotConstants.kL2Position);
        SmartDashboard.putNumber("Pivot/L3 Position", PivotConstants.kL3Position);
        SmartDashboard.putNumber("Pivot/L4 Position", PivotConstants.kL4Position);
    }
}
