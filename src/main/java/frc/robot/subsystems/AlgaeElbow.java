package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
//SparkMax motor controller
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
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
  private AbsoluteEncoder m_encoder;
  private AbsoluteEncoderConfig m_encoderConfig;
  private SparkMaxConfig m_motorConfig;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double targetPosition = 0;
  //boolean goingDown = false;
  //double currentVelo = 0.0;

  /** Creates a new AlgaeElbow. */
  public AlgaeElbow() {

    m_motor = new SparkMax(deviceID, MotorType.kBrushless);
    m_controller = m_motor.getClosedLoopController();
    m_encoder = m_motor.getAbsoluteEncoder();
    m_motorConfig = new SparkMaxConfig();

    m_motorConfig = new SparkMaxConfig();
    m_motorConfig.voltageCompensation(12);
    m_motorConfig.smartCurrentLimit(30,60,200);
    m_motorConfig.inverted(false);
    m_motorConfig.idleMode(IdleMode.kBrake);
    m_motorConfig.closedLoopRampRate(3);

    //m_motorConfig.absoluteEncoder.positionConversionFactor(100);

    m_motorConfig.softLimit.reverseSoftLimit(0);
    m_motorConfig.softLimit.reverseSoftLimitEnabled(true);

    m_motorConfig.softLimit.forwardSoftLimit(26);
    m_motorConfig.softLimit.forwardSoftLimitEnabled(true);

    //m_encoder.setPosition(0.0);
    m_encoderConfig.zeroOffset(67.5);

    m_motorConfig.closedLoop
      //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(-0.5, .5)
      // Set PID values for velocity control in slot 1
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {

    //double previousVelo = currentVelo;
    //currentVelo = m_encoder.getVelocity();

    boolean hitBottom = false;
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("algae elbow Position", m_encoder.getPosition());
    SmartDashboard.putNumber("AE applied output",m_motor.getAppliedOutput());
    SmartDashboard.putNumber("AE bus voltage",m_motor.getBusVoltage());
    SmartDashboard.putNumber("AE output current",m_motor.getOutputCurrent());

    // if(goingDown && currentVelo > previousVelo){
    //   targetPosition = -38.0;
    //   m_encoder.setPosition(-38.0);
    //   goingDown = false;
    //   hitBottom = true;
    // }

    
    SmartDashboard.putNumber("Velocity", m_encoder.getVelocity());
    SmartDashboard.putBoolean("Hit Bottom", hitBottom);
    //m_controller.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

  }
  public void up(){
    m_motor.set(0.75);
    //targetPosition = 0.0;
    //goingDown = false;
  }
  public void down(){
    m_motor.set(-0.45);
    //targetPosition = -38.0; 
    //goingDown = true;
  }
  public void stop(){
    m_motor.stopMotor();
  }
}
