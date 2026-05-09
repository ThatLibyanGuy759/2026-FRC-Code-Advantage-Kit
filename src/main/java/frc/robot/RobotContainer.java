// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.util.ElasticDashboard;
import frc.robot.drive.TunerConstants;
import frc.robot.elevator.ElevatorCommand;
import frc.robot.elevator.ElevatorConstants;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.hopper.HopperCommand;
import frc.robot.hopper.HopperConstants;
import frc.robot.hopper.HopperExtendCommandGroup;
import frc.robot.hopper.HopperFeedShootCommand;
import frc.robot.hopper.HopperIOKraken;
import frc.robot.hopper.HopperIOSim;
import frc.robot.hopper.HopperRetractCommandGroup;
import frc.robot.hopper.HopperSubsystem;
import frc.robot.hopper.HopperToggleCommand;
import frc.robot.intake.IntakeAutoStartCommandGroup;
// import frc.robot.intake.IntakeAutoStartCommandGroup;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.OutakeCommand;
import frc.robot.roller.RollerSubsystem;
import frc.robot.drive.AlignToTrench;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.AlignRotationToHubOdometry;
import frc.robot.vision.LimeLightSubsystem;
import frc.robot.vision.ResetOdometryLimelight;
import frc.robot.vision.ShootFromHubDistance;
import frc.robot.shooter.KickerCommandGroup;
import frc.robot.shooter.KickerRollerCommand;
import frc.robot.shooter.KickerSubsystem;
import frc.robot.shooter.LeftShooterSubsystem;
import frc.robot.shooter.RightShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    boolean isPressed;
    private final LeftShooterSubsystem m_leftShooterSubsystem = new LeftShooterSubsystem();
    private final RightShooterSubsystem m_rightShooterSubsystem = new RightShooterSubsystem();
    private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
    private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem( Robot.isSimulation() ? new HopperIOSim() : new HopperIOKraken());
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LimeLightSubsystem m_limeLightSubsystem = new LimeLightSubsystem(drivetrain);
    private final ElasticDashboard elastic_dashboard = new frc.robot.util.ElasticDashboard(drivetrain, m_limeLightSubsystem);
    private final SendableChooser<Command> m_autoChooser;
    BooleanSupplier isShooting = () -> m_kickerSubsystem.getStatorCurrentAmps() > 15.0;
    double speedMultiplier;

    public RobotContainer() {
        NamedCommands.registerCommand("ResetOdometryLimelight", new ResetOdometryLimelight(drivetrain));
        NamedCommands.registerCommand("IntakeHopper", new IntakeAutoStartCommandGroup(m_intakeSubsystem, m_hopperSubsystem));
        NamedCommands.registerCommand("AlignRotationToHubOdometry", new AlignRotationToHubOdometry( 
            drivetrain,
            m_limeLightSubsystem,
            () -> MathUtil.applyDeadband(joystick.getLeftY(), 0.10) * MaxSpeed,
            () -> MathUtil.applyDeadband(joystick.getLeftX(), 0.10) * MaxSpeed
        ).withTimeout(1.0));
        NamedCommands.registerCommand("HopperOutCommand", new HopperCommand(m_hopperSubsystem, HopperConstants.fullyExtended).withTimeout(0.5));
        NamedCommands.registerCommand("ShootFromHubDistance", new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem));
        NamedCommands.registerCommand("RunKickerRollerHopper", new KickerCommandGroup(m_kickerSubsystem, m_rollerSubsystem,m_intakeSubsystem, m_hopperSubsystem));

        m_autoChooser = AutoBuilder.buildAutoChooser();
        
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                boolean isPressed = joystick.leftBumper().getAsBoolean();
                speedMultiplier = isPressed ? 0.45 : 0.8; 
                
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedMultiplier) 
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedMultiplier) 
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        joystick.rightTrigger().whileTrue(
            new ParallelCommandGroup(
                new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem),
                
                new SequentialCommandGroup(
                    new AlignRotationToHubOdometry(
                        drivetrain,
                        m_limeLightSubsystem,
                        () -> MathUtil.applyDeadband(-joystick.getLeftY(), 0.10) * MaxSpeed,
                        () -> MathUtil.applyDeadband(-joystick.getLeftX(), 0.10) * MaxSpeed
                    ) //.withTimeout(1.25),

                    //drivetrain.applyRequest(() -> brake).withTimeout(3.0)
                )
            )
        ).onFalse(new HopperCommand(m_hopperSubsystem, HopperConstants.fullyExtended));

        joystick.povUp().whileTrue(
            new ParallelCommandGroup(
                new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem),
                new IntakeCommand(m_intakeSubsystem, 12)
            )
        );

        joystick.rightBumper().whileTrue(new KickerCommandGroup(m_kickerSubsystem, m_rollerSubsystem, m_intakeSubsystem, m_hopperSubsystem)).onFalse(new HopperExtendCommandGroup(m_hopperSubsystem));

        joystick.povLeft().onTrue(new ResetOdometryLimelight(drivetrain));
        joystick.povDown().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
        joystick.leftTrigger().whileTrue(new IntakeCommand(m_intakeSubsystem, 12));
        joystick.x().whileTrue(new OutakeCommand(m_intakeSubsystem, m_rollerSubsystem, 12));
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.y().onTrue(new HopperExtendCommandGroup(m_hopperSubsystem));
        joystick.b().onTrue(new HopperRetractCommandGroup(m_hopperSubsystem));
        // joystick.b().onTrue(new ElevatorCommand(m_elevatorSubsystem, ElevatorConstants.fullyExtended));

        // joystick.a().onTrue(new HopperCommand(m_hopperSubsystem, HopperConstants.fullyExtended));
        // joystick.y().onTrue(new HopperCommand(m_hopperSubsystem, HopperConstants.fullyRetracted));
        // joystick.povRight().onTrue(new InstantCommand(() ->  m_hopperSubsystem.zeroHopper()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //normal trench autos
        // return new PathPlannerAuto("RightGreedyTrenchCenter2CycleAuto");
        // return new PathPlannerAuto("LeftGreedyTrenchCenter2CycleAuto");
        
        //middle autos
        return new PathPlannerAuto("BackAuto");
        // return new PathPlannerAuto("BackAutoDepot");

        //heist autos 🤑🤑
        // return new PathPlannerAuto("LLeftDELAYEDTrenchCenter2CycleAuto");
        // return new PathPlannerAuto("RightDELAYEDTrenchCenter2CycleAuto");
    }
}