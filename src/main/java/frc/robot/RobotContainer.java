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
import frc.robot.commands.elevator.ElevatorPositionCommandBase;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // max angular velocity

    /* Swerve drive platform setup */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joysticks = new CommandXboxController(1);
    private final CommandXboxController joysticksb = new CommandXboxController(2);

    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Shooter shooter = new Shooter();
    private final Pivot m_Pivot = new Pivot();
    private final Elevator m_elevator = new Elevator();
    private final Climber m_climber = new Climber();

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private final double kElevatorGravityCompensation = 0.03;
    private final double kPositionGravityCompensation = -0.40; // Adjust this value based on testing

    private final SendableChooser<Command> autoChooser;

    private final ElevatorToL2Position m_elevatorToL2Position;
    private final ElevatorToL3Position m_elevatorToL3Position;
    private final ElevatorToL4Position m_elevatorToL4Position;

    private final PivotSetPositionCommand m_pivotToL2L3;
    private final PivotSetPositionCommand m_pivotTo0;



    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Blue");
        
        // Initialize the base command with common dependencies
        ElevatorPositionCommandBase.initialize(m_elevator, m_request);
        ElevatorToL4Position.initialize(shooter);
        
        // Create command instances
        m_elevatorToL2Position = new ElevatorToL2Position();
        m_elevatorToL3Position = new ElevatorToL3Position();
        m_elevatorToL4Position = new ElevatorToL4Position();

        // Create Pivot position command (adjust the position value as needed)
        m_pivotToL2L3 = new PivotSetPositionCommand(m_Pivot, m_request, -2.066);
        m_pivotTo0 = new PivotSetPositionCommand(m_Pivot, m_request, 0); // Example target position
        // Example target position

        NamedCommands.registerCommand("PivotTarget", m_pivotToL2L3);

        NamedCommands.registerCommand("L2Position", m_elevatorToL2Position);
        NamedCommands.registerCommand("L3Position", m_elevatorToL3Position);
        NamedCommands.registerCommand("L4Position", m_elevatorToL4Position);
        
        configureBindings();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0)
        ));
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0)
        ));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joysticks.rightTrigger().whileTrue(new RunCommand(() -> shooter.shoot(1.0), shooter))
                                .onFalse(new InstantCommand(() -> shooter.shoot(0.0), shooter));

        // Elevator default command
        m_elevator.setDefaultCommand(
            new ElevatorJoystickCommand(m_elevator, joysticks, kElevatorGravityCompensation)
        );

        // Elevator position control with buttons
        joysticks.a().whileTrue(m_elevatorToL2Position);
        joysticks.b().whileTrue(m_elevatorToL3Position);
        joysticks.y().whileTrue(m_elevatorToL4Position);

        //Climber Control
        joysticks.x().whileTrue(m_climber.climberControl());

        // Position default command
        m_Pivot.setDefaultCommand(
            new PositionJoystickCommand(m_Pivot, joysticks, kPositionGravityCompensation)
        );

        joysticksb.rightTrigger().onTrue(m_pivotToL2L3);
        joysticksb.leftTrigger().onTrue(m_pivotTo0);

        //Shooter Control
        joysticksb.leftBumper().whileTrue(shooter.shooterIntakeControl());
        joysticksb.rightBumper().onTrue(shooter.shooterOutakeControl().withTimeout(1.5));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
