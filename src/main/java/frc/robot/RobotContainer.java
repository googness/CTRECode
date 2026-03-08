// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Shooter shooter = new Shooter();

    public final Intake intake = new Intake();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));








        // Run the Flywheels


        // joystick.back().and(joystick.a())
        // .onTrue(new InstantCommand(
        //     () -> { shooter.runFlywheel(); }
        // ))
        // .onFalse(new InstantCommand(
        //     () -> { shooter.stopFlywheel(); }
        // ));


        // joystick.leftBumper()
        //     .toggleOnTrue(new InstantCommand(
        //         () -> { shooter.runFlywheel(); }
        //     ))
        //     .toggleOnFalse(new InstantCommand(
        //         () -> { shooter.stopFlywheel(); }
        //     ));


        // // Run the feeder
        // joystick.leftTrigger().onTrue(new InstantCommand(() -> { shooter.startFeed();}
        // ))
        // .onFalse(new InstantCommand(() -> { shooter.stopFeed();}));


        //         // Run the Intake
        // joystick.rightBumper().onTrue(new InstantCommand(() -> { intake.startIntake();}
        // ))
        // .onFalse(new InstantCommand(() -> { intake.stopIntake();}));


        // // Run the Belt
        // joystick.rightTrigger().onTrue(new InstantCommand(() -> { intake.startBelt();}
        // ))
        // .onFalse(new InstantCommand(() -> { intake.stopBelt();}));

        // // Reverse the belt
        // joystick.start().onTrue(new InstantCommand(() -> {intake.reverseBelt();}
        // ))
        // .onFalse(new InstantCommand(() -> {intake.stopBelt();}));



        // Extend Hopper
     

        // Extend Hood


        // joystick.a().onTrue(new InstantCommand(() -> { shooter.runFlywheel(49); }))
        // .onFalse(new InstantCommand(() -> { shooter.stop(); }));


        joystick.a().toggleOnTrue(shooter.ShootRpsCmd());
        joystick.rightTrigger().whileTrue(shooter.FeederRpsCmd());

        joystick.povUp().onTrue(new InstantCommand(() -> { shooter.extendHood();}));
        joystick.povDown().onTrue(new InstantCommand(() -> { shooter.retractHood();}));



        joystick.back()
            .toggleOnTrue(
                shooter.ShootRpsCmd()
                .alongWith(intake.RunClearBeltCmd())
                .alongWith(new WaitCommand(1)
                    .andThen(
                        shooter.FeederRpsCmd()
                            .alongWith(intake.RunBeltCmd())
                            .alongWith(
                                new WaitCommand(3.5)
                                    .andThen(() -> intake.retractHopper())
                                    .andThen(intake.RunClearIntakeCmd())
                            )
                        ))
                
            );



        joystick.leftTrigger().whileTrue(intake.RunBeltCmd());
        joystick.rightBumper()
            .whileTrue(intake.RunIntakeCmd());

        joystick.povRight().onTrue(new InstantCommand(() -> { intake.extendHopper();}));
        joystick.povLeft().onTrue(new InstantCommand(() -> { intake.retractHopper();}));






        // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
