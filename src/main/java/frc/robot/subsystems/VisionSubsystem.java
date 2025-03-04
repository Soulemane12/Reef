package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.LimelightHelpers;
import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera;
    private final PIDController alignPID;
    private final PIDController distancePID;

    public VisionSubsystem() {
        photonCamera = new PhotonCamera("Limelight");
        
        // PID for horizontal alignment (turning)
        alignPID = new PIDController(kAlignP, kAlignI, kAlignD);
        alignPID.setTolerance(kAlignToleranceDegrees);
        
        // PID for distance control
        distancePID = new PIDController(kDistanceP, kDistanceI, kDistanceD);
        distancePID.setTolerance(kDistanceToleranceMeters);
        
        // Set Limelight LED mode to be controlled by pipeline
        LimelightHelpers.setLEDMode_PipelineControl(kLimelightName);
    }

    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(kLimelightName) || photonCamera.getLatestResult().hasTargets();
    }

    public double getTargetYaw() {
        if (LimelightHelpers.getTV(kLimelightName)) {
            return LimelightHelpers.getTX(kLimelightName);
        }
        
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    public double getTargetDistance() {
        if (LimelightHelpers.getTV(kLimelightName)) {
            // Calculate distance using pitch angle
            double pitch = LimelightHelpers.getTY(kLimelightName);
            return PhotonUtils.calculateDistanceToTargetMeters(
                kCameraHeightMeters,
                kTargetHeightMeters,
                kCameraPitchRadians,
                Math.toRadians(pitch)
            );
        }
        
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        }
        return 0.0;
    }

    public double calculateAlignmentSpeed() {
        double error = getTargetYaw();
        return alignPID.calculate(error, 0.0);
    }

    public double calculateDistanceSpeed() {
        double currentDistance = getTargetDistance();
        return distancePID.calculate(currentDistance, kDesiredDistanceMeters);
    }

    public boolean isAligned() {
        return alignPID.atSetpoint() && distancePID.atSetpoint();
    }

    public void resetPIDs() {
        alignPID.reset();
        distancePID.reset();
    }

    @Override
    public void periodic() {
        // Log vision data to SmartDashboard
        if (hasValidTarget()) {
            double distance = getTargetDistance();
            double yaw = getTargetYaw();
            System.out.printf("Target Distance: %.2fm, Yaw: %.2f°%n", 
                            distance, yaw);
        }
    }
} 