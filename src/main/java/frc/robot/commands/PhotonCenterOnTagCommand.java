package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PhotonCenterOnTagCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final PhotonCamera camera;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    public PhotonCenterOnTagCommand(CommandSwerveDrivetrain drivetrain, String cameraName) {
        m_drivetrain = drivetrain;
        camera = new PhotonCamera(cameraName);
        addRequirements(drivetrain);
        
        // Tune these PID values for your robot
        xController = new PIDController(0.1, 0, 0);
        yController = new PIDController(0.1, 0, 0);
        rotationController = new PIDController(0.1, 0, 0);
        
        rotationController.enableContinuousInput(-180, 180);
    }
    
    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (!result.hasTargets()) {
            m_drivetrain.setControl(new ChassisSpeeds(0, 0, 0));
            return;
        }
        
        var target = result.getBestTarget();
        
        // Calculate speeds using PID controllers
        double xSpeed = -yController.calculate(target.getPitch(), 0);
        double ySpeed = -xController.calculate(target.getYaw(), 0);
        double rotationSpeed = -rotationController.calculate(target.getYaw(), 0);
        
        // Apply speeds to drivetrain
        m_drivetrain.setControl(
            m_drivetrain.applyRequest(() -> 
                new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed)
            ).withIsOpenLoop(true)
        );
    }
    
    @Override
    public boolean isFinished() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) return false;
        
        var target = result.getBestTarget();
        return Math.abs(target.getYaw()) < 1.0 && 
               Math.abs(target.getPitch()) < 1.0;
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new ChassisSpeeds(0, 0, 0));
    }
