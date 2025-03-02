// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {

    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);


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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final CoralIntake coralIntake = new CoralIntake();
    private final AlgaeElbow algaeElbow = new AlgaeElbow();
    private final CoralElbow coralElbow = new CoralElbow();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("SimpleAuto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));

        // driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        /*********************************************************************************************/
        /**********************         DRIVER CONROLLER                ******************************/                          
        driverController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //driverController.x().whileTrue(new InstantCommand(() -> climber.out(), climber));
        //driverController.x().onFalse(new InstantCommand(() -> climber.stop(), climber));

        driverController.b().whileTrue(new InstantCommand(() -> climber.climb(), climber));
        driverController.b().onFalse(new InstantCommand(() -> climber.stop(), climber));

        driverController.a().whileTrue(new InstantCommand(() -> algaeElbow.down(), algaeElbow));
        driverController.a().onFalse(new InstantCommand(() -> algaeElbow.stop(), algaeElbow));

        driverController.y().whileTrue(new InstantCommand(() -> algaeElbow.up(), algaeElbow));
        driverController.y().onFalse(new InstantCommand(() -> algaeElbow.stop(), algaeElbow));

        driverController.rightBumper().whileTrue(new InstantCommand(() -> algaeIntake.in(), algaeIntake));
        driverController.rightBumper().onFalse(new InstantCommand(() -> algaeIntake.hold(), algaeIntake));

        driverController.leftBumper().whileTrue(new InstantCommand(() -> algaeIntake.out(), algaeIntake));
        driverController.leftBumper().onFalse(new InstantCommand(() -> algaeIntake.stop(), algaeIntake));
        /*********************************************************************************************/

        /*********************************************************************************************/
        /**********************         OPERATOR CONROLLER              ******************************/ 
        //operatorController.x().debounce(.5, Debouncer.DebounceType.kBoth).onTrue(new RunCommand(() -> elevator.up(), elevator));
        operatorController.x().whileTrue(new InstantCommand(() -> elevator.upManual(), elevator));
        operatorController.x().onFalse(new InstantCommand(() -> elevator.stop(), elevator));

        //operatorController.b().debounce(.5, Debouncer.DebounceType.kBoth).onTrue(new RunCommand(() -> elevator.down(), elevator));
        operatorController.b().whileTrue(new InstantCommand(() -> elevator.downManual(), elevator));
        operatorController.b().onFalse(new InstantCommand(() -> elevator.stop(), elevator));

        operatorController.a().whileTrue(new InstantCommand(() -> coralElbow.down(), coralElbow));
        //operatorController.a().onFalse(new InstantCommand(() -> coralElbow.stop(), coralElbow));

        operatorController.y().whileTrue(new InstantCommand(() -> coralElbow.up(), coralElbow));
        //operatorController.y().onFalse(new InstantCommand(() -> coralElbow.stop(), coralElbow));

        operatorController.rightBumper().whileTrue(new InstantCommand(() -> coralIntake.in(), coralIntake));
        operatorController.rightBumper().onFalse(new InstantCommand(() -> coralIntake.stop(), coralIntake));

        operatorController.leftBumper().whileTrue(new InstantCommand(() -> coralIntake.out(), coralIntake));
        operatorController.leftBumper().onFalse(new InstantCommand(() -> coralIntake.stop(), coralIntake));
        /***************************************************************************************************** */

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
