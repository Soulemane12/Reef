package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera;
    private final PIDController alignPID;
    private final double CAMERA_HEIGHT_METERS = 0.6; // Adjust based on your robot
    private final double TARGET_HEIGHT_METERS = 1.0; // Adjust based on AprilTag height
    private final double CAMERA_PITCH_RADIANS = Math.toRadians(30); // Adjust based on your camera
    private final double ALIGN_TOLERANCE = 1.0;

    public VisionSubsystem() {
        photonCamera = new PhotonCamera("Limelight");
        alignPID = new PIDController(0.02, 0.0, 0.0); // You'll need to tune these values
        alignPID.setTolerance(ALIGN_TOLERANCE);
    }

    public double getLimelightTx() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    }

    public double getAprilTagYaw() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    public double calculateAlignmentSpeed() {
        double tx = getLimelightTx();
        double tagYaw = getAprilTagYaw();
        double error = (tx + tagYaw) / 2;

        return alignPID.calculate(error, 0.0);
    }
} 