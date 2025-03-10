package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//SparkMax motor controller
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

public class CoralElbow extends SubsystemBase {
  private static final int deviceID = 4;
  private SparkMax m_motor;
  private SparkClosedLoopController m_controller;
  private RelativeEncoder m_encoder;
  private SparkMaxConfig m_motorConfig;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double targetPosition = 0;
  /** Creates a new Climber. */
  public CoralElbow() {

    m_motor = new SparkMax(deviceID, MotorType.kBrushless);
    m_controller = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();
    m_motorConfig = new SparkMaxConfig();

    m_motorConfig = new SparkMaxConfig();
    m_motorConfig.voltageCompensation(12);
    m_motorConfig.smartCurrentLimit(30,60,200);
    m_motorConfig.inverted(true);
    m_motorConfig.idleMode(IdleMode.kBrake);
    m_motorConfig.closedLoopRampRate(4);

    m_motorConfig.softLimit.reverseSoftLimit(-27);
    m_motorConfig.softLimit.reverseSoftLimitEnabled(true);

    m_motorConfig.softLimit.forwardSoftLimit(0);
    m_motorConfig.softLimit.forwardSoftLimitEnabled(true);

    m_encoder.setPosition(0.0);

    m_motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(0.025)
      .i(0)
      .d(0)
      .outputRange(-0.4, .4)
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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("coral elbow Position", m_encoder.getPosition());

    m_controller.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  public void up(){

    if(targetPosition <= -9){
      targetPosition = targetPosition + 9; 
    }
   
  }
  public void down(){
    if(targetPosition >= -18){
      targetPosition = targetPosition - 9; 
    }
  }
  public void stop(){
    m_motor.stopMotor();
  }
}
