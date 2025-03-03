package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.VisionConstants;

public class VisionUtils {
    /**
     * Calculate the distance to the target using the target's y angle
     */
    public static double calculateDistanceToTarget(double targetYDegrees) {
        double targetYRadians = Math.toRadians(targetYDegrees);
        return (VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.CAMERA_HEIGHT_METERS) /
               Math.tan(targetYRadians + VisionConstants.CAMERA_PITCH_RADIANS);
    }

    /**
     * Calculate the robot's pose relative to the target
     */
    public static Transform2d calculateTargetRelativePose(double tx, double ty, double ta) {
        double distance = calculateDistanceToTarget(ty);
        double xOffset = distance * Math.tan(Math.toRadians(tx));
        
        return new Transform2d(
            distance, xOffset,
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(tx)
        );
    }

    /**
     * Check if the target is at a valid distance for tracking
     */
    public static boolean isTargetValid(double targetArea) {
        return targetArea > VisionConstants.MIN_TARGET_AREA;
    }
} 