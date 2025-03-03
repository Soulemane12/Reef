package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CenterOnTagCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    public CenterOnTagCommand(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        
        // Tune these PID values for your robot
        xController = new PIDController(0.1, 0, 0);
        yController = new PIDController(0.1, 0, 0);
        rotationController = new PIDController(0.1, 0, 0);
        
        // Set the rotation controller to continuous since -180 and 180 are the same angle
        rotationController.enableContinuousInput(-180, 180);
    }
    
    @Override
    public void execute() {
        // Get target data from Limelight
        double tx = LimelightHelpers.getTX("");  // Empty string for default Limelight
        double ty = LimelightHelpers.getTY("");
        boolean hasTarget = LimelightHelpers.getTV("");
        
        if (!hasTarget) {
            m_drivetrain.setControl(new ChassisSpeeds(0, 0, 0));
            return;
        }
        
        // Calculate speeds using PID controllers
        double xSpeed = -yController.calculate(ty, 0);  // Forward/backward based on Y offset
        double ySpeed = -xController.calculate(tx, 0);  // Left/right based on X offset
        double rotationSpeed = -rotationController.calculate(tx, 0);  // Rotation to center target
        
        // Apply speeds to drivetrain
        m_drivetrain.setControl(
            m_drivetrain.applyRequest(() -> 
                new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed)
            ).withIsOpenLoop(true)
        );
    }
    
    @Override
    public boolean isFinished() {
        // End command when target is centered (within tolerance)
        return Math.abs(LimelightHelpers.getTX("")) < 1.0 && 
               Math.abs(LimelightHelpers.getTY("")) < 1.0;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        m_drivetrain.setControl(new ChassisSpeeds(0, 0, 0));
    }
} 