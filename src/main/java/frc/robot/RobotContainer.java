package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.elevator.ElevatorJoystickCommand;
import frc.robot.commands.position.PivotSetPositionCommand;
import frc.robot.commands.position.PositionJoystickCommand;
import frc.robot.commands.elevator.ElevatorToL2Position;
import frc.robot.commands.elevator.ElevatorToL3Position;
import frc.robot.commands.elevator.ElevatorToL4Position;
import frc.robot.commands.elevator.ElevatorTo0Position;
import frc.robot.commands.elevator.ElevatorToPoint0Position;

import frc.robot.commands.elevator.ElevatorPositionCommandBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.CenterOnTagCommand;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Reduce deadband to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driver = new CommandXboxController(0); //driver
    private final CommandXboxController operator = new CommandXboxController(1); //operator

    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Shooter shooter = new Shooter();
    private final Pivot m_Pivot = new Pivot();
    private final Elevator m_elevator = new Elevator();
    private final Climber m_climber = new Climber();

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private final double kElevatorGravityCompensation = 0.04;
    private final double kPositionGravityCompensation = -0.6; // Increased gravity compensation

    private final SendableChooser<Command> autoChooser;

    private final ElevatorToL2Position m_elevatorToL2Position;
    private final ElevatorToL3Position m_elevatorToL3Position;
    private final ElevatorToL4Position m_elevatorToL4Position;
    private final ElevatorTo0Position m_elevatorTo0Position;
    private final ElevatorToPoint0Position m_elevatorToPoint0Position;


    private final PivotSetPositionCommand m_pivotToL2;
    private final PivotSetPositionCommand m_pivotToL3;

    private final PivotSetPositionCommand m_pivotTo0;
    private final PivotSetPositionCommand m_pivotToL4;
    private final PivotSetPositionCommand m_pivotToParallel;
    private final PivotSetPositionCommand m_pivotToIN;




    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Red");
        
        // Initialize the base command with common dependencies
        ElevatorPositionCommandBase.initialize(m_elevator, m_request);
        ElevatorToL4Position.initialize(shooter);
        
        // Create command instances
        m_elevatorToL2Position = new ElevatorToL2Position();
        m_elevatorToL3Position = new ElevatorToL3Position();
        m_elevatorToL4Position = new ElevatorToL4Position();
        m_elevatorTo0Position = new ElevatorTo0Position();
        m_elevatorToPoint0Position = new ElevatorToPoint0Position();

        // Set elevator to "fake" zero position on robot init
        m_elevatorToPoint0Position.schedule();

        // Compute the adjusted setpoints with gravity compensation
        double adjustedL2 = PivotConstants.kPivotL2Position + 
            (kPositionGravityCompensation * Math.sin(PivotConstants.kPivotL2Position)) + 
            (0.15 * Math.cos(PivotConstants.kPivotL2Position));
        
        double adjustedL3 = PivotConstants.kPivotL3Position + 
            (kPositionGravityCompensation * Math.sin(PivotConstants.kPivotL3Position)) + 
            (0.15 * Math.cos(PivotConstants.kPivotL3Position));
        
        double adjustedL0 = PivotConstants.kPivotL0Position + 
            (kPositionGravityCompensation * Math.sin(PivotConstants.kPivotL0Position)) + 
            (0.15 * Math.cos(PivotConstants.kPivotL0Position));
        
        double adjustedL4 = PivotConstants.kPivotL4Position + 
            (kPositionGravityCompensation * Math.sin(PivotConstants.kPivotL4Position)) + 
            (0.15 * Math.cos(PivotConstants.kPivotL4Position));
        
        double adjustedParallel = PivotConstants.kPivotParallelPosition + 
            (kPositionGravityCompensation * Math.sin(PivotConstants.kPivotParallelPosition)) + 
            (0.15 * Math.cos(PivotConstants.kPivotParallelPosition));
        
        double adjustedin = PivotConstants.kPivotInPosition + 
            (kPositionGravityCompensation * Math.sin(PivotConstants.kPivotInPosition)) + 
            (0.15 * Math.cos(PivotConstants.kPivotInPosition));

        // Now create your pivot commands with the adjusted setpoints
        m_pivotToL2 = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL2);
        m_pivotToL3 = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL3);

        m_pivotTo0   = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL0);
        m_pivotToL4   = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL4);
        m_pivotToParallel = new PivotSetPositionCommand(m_Pivot, m_request, adjustedParallel);
     
        m_pivotToIN = new PivotSetPositionCommand(m_Pivot, m_request, adjustedin);



        NamedCommands.registerCommand("PivotTarget", m_pivotToL2);

        NamedCommands.registerCommand("L2Position", m_elevatorToL2Position);
        NamedCommands.registerCommand("L3Position", m_elevatorToL3Position);
        NamedCommands.registerCommand("L4Position", m_elevatorToL4Position);
        
        configureBindings();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }


    private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        driver.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.10).withVelocityY(0))
        );
        driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.10).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        

        // Elevator default command
      //  m_elevator.setDefaultCommand(
       //     new ElevatorJoystickCommand(m_elevator, driver, kElevatorGravityCompensation)
       // );

        // Elevator position control with buttons
      //  operator.a().onTrue(m_elevatorToL2Position);
      //  operator.b().onTrue(m_elevatorToL3Position);
     //   operator.y().onTrue(m_elevatorToL4Position);
     //   operator.pov(180).onTrue(m_elevatorToPoint0Position);

        //Climber Control
        //operator.x().onTrue(m_elevatorTo0Position);

        // Position default command
        m_Pivot.setDefaultCommand(
            new PositionJoystickCommand(m_Pivot, driver, kPositionGravityCompensation)
        );

       // operator.rightTrigger().onTrue(m_pivotToL2);
       // operator.leftTrigger().onTrue(m_pivotTo);
        
        // Parallel-Elevator-Pivot sequences
        operator.pov(180).onTrue(Commands.parallel(m_elevatorTo0Position, m_pivotTo0));
        operator.pov(0).onTrue(Commands.sequence(
            m_pivotToIN,
            Commands.waitSeconds(0.5),
            m_elevatorTo0Position
        ));

        operator.a().onTrue(Commands.sequence(
            m_pivotToL2,
            Commands.waitSeconds(0.5),
            m_elevatorToL2Position
        ));
        operator.b().onTrue(Commands.sequence(
            m_pivotToL3,
            Commands.waitSeconds(0.5),
            m_elevatorToL3Position
        ));
        operator.y().onTrue(Commands.sequence(
            m_pivotToL4,
            Commands.waitSeconds(0.5),
            m_elevatorToL4Position
        ));
        operator.x().onTrue(m_pivotToParallel);
      



        


        //Shooter Control
        operator.leftTrigger().whileTrue(shooter.shooterIntakeControl())
        operator.rightTrigger().onTrue(shooter.shooterOutakeControl().withTimeout(0.75));

        // Add centering command to a button
        driver.rightBumper().whileTrue(new CenterOnTagCommand(drivetrain));

        // Add a button to toggle the Limelight LED
        driver.leftTrigger().onTrue(Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink("")))
                    .onFalse(Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceOff("")));

        // Add a button to switch Limelight pipelines
        driver.rightTrigger().onTrue(Commands.runOnce(() -> {
            int currentPipeline = (int)LimelightHelpers.getCurrentPipelineIndex("");
            LimelightHelpers.setPipelineIndex("", (currentPipeline + 1) % 2); // Toggle between pipeline 0 and 1
        }));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void updateVisionMeasurements() {
        if (LimelightHelpers.getTV("")) {
            // Get the robot's pose from Limelight
            double[] botpose = LimelightHelpers.getBotPose_wpiBlue("");  // Use _wpiRed for red alliance
            
            if (botpose.length >= 6) {  // Make sure we got valid data
                Pose2d visionPose = new Pose2d(
                    botpose[0], 
                    botpose[1],
                    Rotation2d.fromDegrees(botpose[5])
                );
                
                // Add the vision measurement to the drivetrain
                drivetrain.addVisionMeasurement(
                    visionPose,
                    Timer.getFPGATimestamp() - (botpose[6] / 1000.0)  // Account for latency
                );
            }
        }
    }
}
