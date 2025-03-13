package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

public class AlgaeElbow extends SubsystemBase {
  private static final int deviceID = 11;
  private SparkMax m_motor;
  private SparkClosedLoopController m_controller;
  private RelativeEncoder m_encoderPrimary;
  //private AbsoluteEncoder m_encoderAbsolute;
  private SparkMaxConfig m_motorConfig;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private int[] setpoints = { 0, 10, 20 };
  private int currentSetpoint = 0;

  /** Creates a new AlgaeElbow. */
  public AlgaeElbow() {

    m_motor = new SparkMax(deviceID, MotorType.kBrushless);
    m_controller = m_motor.getClosedLoopController();

    m_encoderPrimary = m_motor.getEncoder();
    //m_encoderAbsolute = m_motor.getAbsoluteEncoder();

    m_motorConfig = new SparkMaxConfig();

    // m_motorConfig.alternateEncoder
    // .countsPerRevolution(8192)
    // .setSparkMaxDataPortConfig();

    m_motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_motorConfig.closedLoop.p(0.1);
    m_motorConfig.closedLoop.i(0);
    m_motorConfig.closedLoop.d(0);
    m_motorConfig.closedLoop.outputRange(-1, 1);
    m_motorConfig.closedLoop.p(0.0001, ClosedLoopSlot.kSlot1);
    m_motorConfig.closedLoop.i(0, ClosedLoopSlot.kSlot1);
    m_motorConfig.closedLoop.d(0, ClosedLoopSlot.kSlot1);
    m_motorConfig.closedLoop.velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1);
    m_motorConfig.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    m_motorConfig.voltageCompensation(12);
    m_motorConfig.smartCurrentLimit(30, 60, 200);
    m_motorConfig.inverted(false);
    m_motorConfig.idleMode(IdleMode.kBrake);
    m_motorConfig.closedLoopRampRate(3);

    m_motorConfig.softLimit.reverseSoftLimit(0);
    m_motorConfig.softLimit.reverseSoftLimitEnabled(false);
    m_motorConfig.softLimit.forwardSoftLimit(40);
    m_motorConfig.softLimit.forwardSoftLimitEnabled(false);

    m_motorConfig.encoder.positionConversionFactor(1);
    m_motorConfig.encoder.velocityConversionFactor(1);

    m_encoderPrimary.setPosition(0.0);
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elbow primary Position", m_encoderPrimary.getPosition());
    //SmartDashboard.putNumber("elbow alternate Position", m_encoderAbsolute.getPosition());
    SmartDashboard.putNumber("elbow Velocity", m_encoderPrimary.getVelocity());
    SmartDashboard.putNumber("elbow Target Position", setpoints[currentSetpoint]);

  }

  public void upManual() {
    m_motor.set(0.75);
  }

  public void downManual() {
    m_motor.set(-0.45);
  }

  public void up() {
    if (currentSetpoint < setpoints.length) {
      currentSetpoint++;
      m_controller.setReference(setpoints[currentSetpoint], ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.1);
    }
  }

  public void down() {
    if (currentSetpoint > 0) {
      currentSetpoint--;
      m_controller.setReference(setpoints[currentSetpoint], ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.1);
    }
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
