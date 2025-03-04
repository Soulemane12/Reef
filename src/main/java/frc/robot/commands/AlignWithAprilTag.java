package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignWithAprilTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final SwerveRequest.RobotCentric turnRequest;
    private final double ALIGNMENT_TOLERANCE = 1.0; // degrees

    public AlignWithAprilTag(CommandSwerveDrivetrain drive, VisionSubsystem vision) {
        this.drivetrain = drive;
        this.visionSubsystem = vision;
        this.turnRequest = new SwerveRequest.RobotCentric();
        addRequirements(drive, vision);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasValidTarget()) {
            double alignmentSpeed = visionSubsystem.calculateAlignmentSpeed();
            drivetrain.setControl(turnRequest.withRotationalRate(alignmentSpeed));
        } else {
            // If no target is visible, stop rotation
            drivetrain.setControl(turnRequest.withRotationalRate(0));
        }
    }

    @Override
    public boolean isFinished() {
        if (!visionSubsystem.hasValidTarget()) {
             return false;
        }
        return Math.abs(visionSubsystem.getTargetYaw()) < ALIGNMENT_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(turnRequest.withRotationalRate(0));
    }
} 