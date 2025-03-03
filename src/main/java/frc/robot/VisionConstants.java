package frc.robot;

public final class VisionConstants {
    // Camera mounting parameters relative to robot center
    public static final double CAMERA_HEIGHT_METERS = 0.5; // Adjust to your robot
    public static final double CAMERA_PITCH_RADIANS = Math.toRadians(0); // Adjust to your mounting angle
    
    // April Tag parameters
    public static final double TARGET_HEIGHT_METERS = 1.45; // Height of April Tags
    
    // Vision processing constants
    public static final double MIN_TARGET_AREA = 0.01; // Minimum area to consider a valid target
    public static final double DESIRED_TARGET_AREA = 0.1; // Desired target area for distance control
    
    // PID Constants for vision alignment
    public static final double VISION_X_P = 0.1;
    public static final double VISION_X_I = 0.0;
    public static final double VISION_X_D = 0.0;
    
    public static final double VISION_Y_P = 0.1;
    public static final double VISION_Y_I = 0.0;
    public static final double VISION_Y_D = 0.0;
    
    public static final double VISION_ROT_P = 0.1;
    public static final double VISION_ROT_I = 0.0;
    public static final double VISION_ROT_D = 0.0;
} 