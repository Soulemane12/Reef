// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ElevatorConstants{
    public static final int kElevatorMotorID1 = 20;
    public static final int kElevatorMotorID2 = 21;
    
    public static final double kL0Position = 0.25;  // Zero/Home position
    public static final double kL2Position = 1.0;   // L2 position
    public static final double kL3Position = 3.0;   // L3 position
    public static final double kL4Position = 4.5;   // L4 position
    
    public static final double kGravityCompensation = 0.04;
    public static final double kPositionTolerance = 0.1;
  }

  public static class PivotConstants {
    public static final int kPivotMotorID = 30;
    
    public static final double kL0Position = 0.75;      // Zero/Home position
    public static final double kL2Position = -1.83;     // L2 position
    public static final double kL3Position = -1.4;      // L3 position
    public static final double kL4Position = -1.5;      // L4 position
    public static final double kParallelPosition = -3.0; // Parallel to ground
    public static final double kInPosition = -2.94;     // In position
    
    public static final double kGravityCompensation = -0.6;
    public static final double kPositionTolerance = 0.05;
    public static final double kCosineCompensation = 0.15;
  }
}
