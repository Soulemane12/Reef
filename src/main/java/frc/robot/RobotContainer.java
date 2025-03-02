package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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

import frc.robot.commands.elevator.ElevatorPositionCommandBase;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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
    private final double kPositionGravityCompensation = -0.30; // Adjust this value based on testing

    private final SendableChooser<Command> autoChooser;

    private final ElevatorToL2Position m_elevatorToL2Position;
    private final ElevatorToL3Position m_elevatorToL3Position;
    private final ElevatorToL4Position m_elevatorToL4Position;
    private final ElevatorTo0Position m_elevatorTo0Position;


    private final PivotSetPositionCommand m_pivotToL2L3;
    private final PivotSetPositionCommand m_pivotTo0;
    private final PivotSetPositionCommand m_pivotToL4;




    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Blue");
        
        // Initialize the base command with common dependencies
        ElevatorPositionCommandBase.initialize(m_elevator, m_request);
        ElevatorToL4Position.initialize(shooter);
        
        // Create command instances
        m_elevatorToL2Position = new ElevatorToL2Position();
        m_elevatorToL3Position = new ElevatorToL3Position();
        m_elevatorToL4Position = new ElevatorToL4Position();
        m_elevatorTo0Position = new ElevatorTo0Position();

            // Define your base setpoints (in radians)
        double L2L3 = -3; 
        double L0 = 0;
        double L4 = -1;

        // Compute the adjusted setpoints by adding gravity compensation.
        // kPositionGravityCompensation is assumed to be a constant like -0.30 (adjust as needed).
        double adjustedL2L3 = L2L3 + (kPositionGravityCompensation * Math.sin(L2L3));
        double adjustedL0   = L0   + (kPositionGravityCompensation * Math.sin(L0));   // sin(0)==0 so remains 0
        double adjustedL4   = L4   + (kPositionGravityCompensation * Math.sin(L4));

        // Now create your pivot commands with the adjusted setpoints
        m_pivotToL2L3 = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL2L3);
        m_pivotTo0   = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL0);
        m_pivotToL4   = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL4);


        NamedCommands.registerCommand("PivotTarget", m_pivotToL2L3);

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
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.rightTrigger().whileTrue(new RunCommand(() -> shooter.shoot(1.0), shooter))
                                .onFalse(new InstantCommand(() -> shooter.shoot(0.0), shooter));

        // Elevator default command
        m_elevator.setDefaultCommand(
            new ElevatorJoystickCommand(m_elevator, driver, kElevatorGravityCompensation)
        );

        // Elevator position control with buttons
        operator.a().onTrue(m_elevatorToL2Position);
        operator.b().onTrue(m_elevatorToL3Position);
        operator.y().onTrue(m_elevatorToL4Position);

        //Climber Control
        operator.x().onTrue(m_elevatorTo0Position);

        // Position default command
        m_Pivot.setDefaultCommand(
            new PositionJoystickCommand(m_Pivot, driver, kPositionGravityCompensation)
        );

        operator.rightTrigger().onTrue(m_pivotToL2L3);
        operator.leftTrigger().onTrue(m_pivotTo0);
        
        operator.leftTrigger().onTrue(m_pivotToL4);
        
        //NEW METHODS FOR TESTING
        operator.x().onTrue(m_pivotTo0.andThen(m_elevatorTo0Position));
        //operator.a().onTrue(m_pivotToL2L3.andThen(m_elevatorToL2Position));
        //operator.b().onTrue(m_pivotToL2L3.andThen(m_elevatorToL3Position));
        //operator.y().onTrue(m_pivotToL4.andThen(m_elevatorToL4Position));
    

        //Shooter Control
        operator.leftBumper().whileTrue(shooter.shooterIntakeControl());
        operator.rightBumper().onTrue(shooter.shooterOutakeControl().withTimeout(0.75));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
