package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignWithAprilTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final SwerveRequest.RobotCentric turnRequest;

    public AlignWithAprilTag(CommandSwerveDrivetrain drive, VisionSubsystem vision) {
        this.drivetrain = drive;
        this.visionSubsystem = vision;
        this.turnRequest = new SwerveRequest.RobotCentric();
        addRequirements(drive, vision);
    }

    @Override
    public void execute() {
        double alignmentSpeed = visionSubsystem.calculateAlignmentSpeed();
        drivetrain.setControl(turnRequest.withRotationalRate(alignmentSpeed));
    }

    @Override
    public boolean isFinished() {
        double error = visionSubsystem.getLimelightTx() + visionSubsystem.getAprilTagYaw();
        return Math.abs(error) < 1.0;}


    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(turnRequest.withRotationalRate(0));
    }
} 