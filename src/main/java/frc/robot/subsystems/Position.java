package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Position extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(ShooterConstants.kShooterMotorID, "ChooChooTrain");

    private double INITIAL_OFFSET = 0;
    private boolean hasInitialized = false;

    public Position() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure feedback and gear ratio if needed
        config.Feedback.SensorToMechanismRatio = 1.0; // Adjust based on gear ratio
        
        // Configure Motion Magic parameters
        config.MotionMagic.withMotionMagicCruiseVelocity(15.0)  // Adjust for desired speed
                          .withMotionMagicAcceleration(30.0)    // Adjust acceleration
                          .withMotionMagicJerk(3000.0);        // Adjust jerk

        // Configure PID values
        config.Slot0.kP = 20.0;  // Adjust based on tuning
        config.Slot0.kI = 0.0;   
        config.Slot0.kD = 0.1;   
        config.Slot0.kS = 0.2;  
        config.Slot0.kV = 1.0;   
        config.Slot0.kA = 0.05;  
        config.Slot0.kG = 0.1;

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
        m_motor.setControl(new MotionMagicVoltage(targetPosition));
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
        double position = (m_motor.getPosition().getValueAsDouble() - INITIAL_OFFSET);
        System.out.println("Shooter Position: " + position);
    }
}
