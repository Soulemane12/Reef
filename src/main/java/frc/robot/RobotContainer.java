package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Position;
import frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Shooter shooter = new Shooter();
    private final Position m_position = new Position();
    private final Elevator m_elevator = new Elevator();

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private final double kGravityCompensation = 0.03;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
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

        // Elevator movement with joystick control
        m_elevator.setDefaultCommand(new RunCommand(
            () -> {
                double rawSpeed = joysticks.getLeftY();
                double speed = applyDeadband(rawSpeed, 0.05);
                
                if (speed == 0) {
                    m_elevator.moveElevator(kGravityCompensation);
                } else if (speed < 0) {
                    m_elevator.moveElevator(-speed + kGravityCompensation);
                } else {
                    m_elevator.moveElevator(-speed);
                }
            },
            m_elevator));

        // Elevator position control with buttons
        joysticks.a().whileTrue(new RunCommand(() -> m_elevator.setPositionWithRequest(m_request.withPosition(1)), m_elevator));
        joysticks.b().whileTrue(new RunCommand(() -> m_elevator.setPositionWithRequest(m_request.withPosition(3)), m_elevator));
        joysticks.y().whileTrue(new RunCommand(() -> m_elevator.setPositionWithRequest(m_request.withPosition(4.5)), m_elevator));

                // Shooter position control with buttons
        joysticks.x().whileTrue(new RunCommand(() -> m_position.setShooterPosition(1.0), m_position));
      //  joysticks.y().whileTrue(new RunCommand(() -> m_position.setShooterPosition(2.5), m_position));
    //    joysticks.b().whileTrue(new RunCommand(() -> m_position.setShooterPosition(4.0), m_position));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0 : value;
    }
}
