package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator_minion elevator = new Elevator_minion();
    private final Climber climber = new Climber();
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final CoralIntake coralIntake = new CoralIntake();
    private final AlgaeElbow algaeElbow = new AlgaeElbow();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("autoShoot", new InstantCommand(() -> coralIntake.auto_shoot(),coralIntake));

        NamedCommands.registerCommand("auto_shootL1", new InstantCommand(() -> coralIntake.score(elevator),coralIntake));
        NamedCommands.registerCommand("auto_shootL2", new InstantCommand(() -> coralIntake.score(elevator),coralIntake));
        NamedCommands.registerCommand("auto_shootL3", new InstantCommand(() -> coralIntake.score(elevator),coralIntake));
        NamedCommands.registerCommand("auto_shootL4", new InstantCommand(() -> coralIntake.score(elevator),coralIntake));

        NamedCommands.registerCommand("autoStop", new InstantCommand(() -> coralIntake.stop(),coralIntake));

        NamedCommands.registerCommand("auto_gotoL1", new InstantCommand(() -> elevator.auto_gotoLevel(1),elevator));
        NamedCommands.registerCommand("auto_gotoL2", new InstantCommand(() -> elevator.auto_gotoLevel(2),elevator));
        NamedCommands.registerCommand("auto_gotoL3", new InstantCommand(() -> elevator.auto_gotoLevel(3),elevator));
        NamedCommands.registerCommand("auto_gotoL4", new InstantCommand(() -> elevator.auto_gotoLevel(4),elevator));

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

        /*********************************************************************************************/
        /**********************         DRIVER CONROLLER                ******************************/                          
        driverController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.rightBumper().whileTrue(new InstantCommand(() -> climber.climb(), climber));
        driverController.rightBumper().onFalse(new InstantCommand(() -> climber.stop(), climber));

        driverController.a().whileTrue(new InstantCommand(() -> algaeElbow.downManual(), algaeElbow));
        driverController.a().onFalse(new InstantCommand(() -> algaeElbow.stop(), algaeElbow));

        driverController.y().whileTrue(new InstantCommand(() -> algaeElbow.upManual(), algaeElbow));
        driverController.y().onFalse(new InstantCommand(() -> algaeElbow.stop(), algaeElbow));

        //driverController.b().whileTrue(new InstantCommand(() -> algaeIntake.in(), algaeIntake));
        driverController.b().toggleOnTrue(new InstantCommand(() -> algaeIntake.in(), algaeIntake));
        driverController.b().onFalse(new InstantCommand(() -> algaeIntake.hold(), algaeIntake));

        driverController.x().whileTrue(new InstantCommand(() -> algaeIntake.out(), algaeIntake));
        driverController.x().onFalse(new InstantCommand(() -> algaeIntake.stop(), algaeIntake));
        /*********************************************************************************************/

        /*********************************************************************************************/
        /**********************         OPERATOR CONROLLER              ******************************/ 
        operatorController.y().whileTrue(new InstantCommand(() -> elevator.up(), elevator));
        //operatorController.y().onFalse(new InstantCommand(() -> elevator.stop(), elevator));

        operatorController.a().whileTrue(new InstantCommand(() -> elevator.down(), elevator));
        //operatorController.a().onFalse(new InstantCommand(() -> elevator.stop(), elevator));

        operatorController.b().whileTrue(new InstantCommand(() -> coralIntake.prep(), coralIntake));
        operatorController.b().onFalse(new InstantCommand(() -> coralIntake.stop(), coralIntake));

        operatorController.x().whileTrue(new InstantCommand(() -> coralIntake.score(elevator), coralIntake));
        operatorController.x().onFalse(new InstantCommand(() -> coralIntake.stop(), coralIntake));

        operatorController.rightBumper().whileTrue(new InstantCommand(() -> coralIntake.retract(), coralIntake));
        operatorController.rightBumper().onFalse(new InstantCommand(() -> coralIntake.stop(), coralIntake));

        /***************************************************************************************************** */

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
