package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera;
    private final PIDController alignPID;

    public VisionSubsystem() {
        photonCamera = new PhotonCamera(VisionConstants.kLimelightName);
        alignPID = new PIDController(
            VisionConstants.kAlignP,
            VisionConstants.kAlignI,
            VisionConstants.kAlignD
        );
        alignPID.setTolerance(VisionConstants.kAlignToleranceDegrees);
    }

    public double getLimelightTx() {
        return NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName).getEntry("tx").getDouble(0.0);
    }

    public double getAprilTagYaw() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    public double calculateAlignmentSpeed() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        double error;
        if (result.hasTargets()) {
            // Use AprilTag yaw if available
            error = result.getBestTarget().getYaw();
        } else {
            // Otherwise use limelight tx
            error = getLimelightTx();
        }
        return alignPID.calculate(error, 0.0);
    }
    

    /**
     * Calculate the distance to the target using camera geometry
     * @return Distance to the target in meters, or -1 if no target is visible
     */
    public double getDistanceToTarget() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kCameraHeightMeters,
                VisionConstants.kTargetHeightMeters,
                VisionConstants.kCameraPitchRadians,
                Math.toRadians(result.getBestTarget().getPitch())
            );
        }
        return -1.0;
    }

    @Override
    public void periodic() {
        // Log vision data to SmartDashboard
        double distance = getDistanceToTarget();
        if (distance > 0) {
            double error = (getLimelightTx() + getAprilTagYaw()) / 2;
            System.out.printf("Target Distance: %.2fm, Alignment Error: %.2f°%n", 
                            distance, error);
        }
    }
} 