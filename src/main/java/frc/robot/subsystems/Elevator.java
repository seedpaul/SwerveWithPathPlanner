package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Elevator extends SubsystemBase {

  private SparkMax RightElevMotor;
  private SparkMax LeftElevMotor;
  private SparkMaxConfig RightElevMotorConfig;
  private SparkMaxConfig LeftElevMotorConfig;
  private static SparkClosedLoopController RightClosedLoopController;
  private RelativeEncoder RightRelativeEncoder;
  private int[] setpoints = { 0, 10, 20, 30, 40, 50 };
  private int currentSetpoint = 0;
  private int targetPosition = 0;

  /** Creates a new Elevator. */
  public Elevator() {

    RightElevMotor = new SparkMax(19, MotorType.kBrushless);
    LeftElevMotor = new SparkMax(17, MotorType.kBrushless);

    RightClosedLoopController = RightElevMotor.getClosedLoopController();
    RightRelativeEncoder = RightElevMotor.getAlternateEncoder();

    RightElevMotorConfig = new SparkMaxConfig();
    LeftElevMotorConfig = new SparkMaxConfig();

    RightElevMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
    RightElevMotorConfig.closedLoop.maxMotion.maxVelocity(300);
    RightElevMotorConfig.closedLoop.maxMotion.maxAcceleration(600);
    RightElevMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(1);
    RightElevMotorConfig.closedLoop.maxMotion.maxAcceleration(500, ClosedLoopSlot.kSlot1);
    RightElevMotorConfig.closedLoop.maxMotion.maxVelocity(6000, ClosedLoopSlot.kSlot1);
    RightElevMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

    RightElevMotorConfig.closedLoop.p(0.4);
    RightElevMotorConfig.closedLoop.i(0);
    RightElevMotorConfig.closedLoop.d(0);
    RightElevMotorConfig.closedLoop.outputRange(-1, 1);
    RightElevMotorConfig.closedLoop.p(0.0001, ClosedLoopSlot.kSlot1);
    RightElevMotorConfig.closedLoop.i(0, ClosedLoopSlot.kSlot1);
    RightElevMotorConfig.closedLoop.d(0, ClosedLoopSlot.kSlot1);
    RightElevMotorConfig.closedLoop.velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1);
    RightElevMotorConfig.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    LeftElevMotorConfig.follow(RightElevMotor, true);

    RightElevMotorConfig.voltageCompensation(12);
    LeftElevMotorConfig.voltageCompensation(12);

    RightElevMotorConfig.smartCurrentLimit(30, 100, 20);
    LeftElevMotorConfig.smartCurrentLimit(30, 100, 20);

    RightElevMotorConfig.inverted(true);
    LeftElevMotorConfig.inverted(true);

    RightElevMotorConfig.idleMode(IdleMode.kBrake);
    LeftElevMotorConfig.idleMode(IdleMode.kBrake);

    RightElevMotorConfig.closedLoopRampRate(4);
    LeftElevMotorConfig.closedLoopRampRate(4);

    RightElevMotorConfig.softLimit.reverseSoftLimit(0);
    RightElevMotorConfig.softLimit.reverseSoftLimitEnabled(false);

    RightElevMotorConfig.softLimit.forwardSoftLimit(40);
    RightElevMotorConfig.softLimit.forwardSoftLimitEnabled(false);

    RightElevMotorConfig.absoluteEncoder.positionConversionFactor(1);
    RightElevMotorConfig.absoluteEncoder.velocityConversionFactor(1);

    RightElevMotor.configure(RightElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    LeftElevMotor.configure(LeftElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.putNumber("elevator Target Position", targetPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator Position", RightRelativeEncoder.getPosition());
    SmartDashboard.putNumber("elevator Velocity", RightRelativeEncoder.getVelocity());
    SmartDashboard.putNumber("elevator Target Position", setpoints[currentSetpoint]);
  }

  public void up() {
    if (currentSetpoint < setpoints.length) {
      currentSetpoint++;
      RightClosedLoopController.setReference(setpoints[currentSetpoint], ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0, 0.1);
    }
  }

  public void down() {
    if (currentSetpoint > 0) {
      currentSetpoint--;
      RightClosedLoopController.setReference(setpoints[currentSetpoint], ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0, 0.1);
    }
  }

  public void stop() {
    RightElevMotor.stopMotor();
  }

  public void hold() {
    RightElevMotor.set(0.05);
  }

  public void upManual() {
    RightElevMotor.set(0.3);
  }

  public void downManual() {
    RightElevMotor.set(-0.2);
  }
}
