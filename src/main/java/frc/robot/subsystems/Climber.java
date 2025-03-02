// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Climber extends SubsystemBase {
  private static final int deviceID = 44;
  private SparkFlex m_motor;
  private SparkClosedLoopController m_controller;
  private RelativeEncoder m_encoder;
  private SparkMaxConfig m_motorConfig;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  /** Creates a new Climber. */
  public Climber() {

    m_motor = new SparkFlex(deviceID, MotorType.kBrushless);
    m_controller = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();
    m_motorConfig = new SparkMaxConfig();

    m_motorConfig = new SparkMaxConfig();
    m_motorConfig.voltageCompensation(12);
    m_motorConfig.smartCurrentLimit(80,120,20);
    m_motorConfig.inverted(true);
    m_motorConfig.idleMode(IdleMode.kBrake);

    m_motorConfig.softLimit.reverseSoftLimit(0);
    m_motorConfig.softLimit.reverseSoftLimitEnabled(true);

    m_motorConfig.softLimit.forwardSoftLimit(120);
    m_motorConfig.softLimit.forwardSoftLimitEnabled(true);

    m_encoder.setPosition(0.0);

    // m_motorConfig.closedLoop
    //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //   // Set PID values for position control. We don't need to pass a closed loop
    //   // slot, as it will default to slot 0.
    //   .p(0.1)
    //   .i(0)
    //   .d(0)
    //   .outputRange(-1, 1)
    //   // Set PID values for velocity control in slot 1
    //   .p(0.0001, ClosedLoopSlot.kSlot1)
    //   .i(0, ClosedLoopSlot.kSlot1)
    //   .d(0, ClosedLoopSlot.kSlot1)
    //   .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    //   .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climber posistion:",m_encoder.getPosition());
  }
  public void out(){
      m_motor.set(-.50);
  }
  public void climb(){
    m_motor.set(.50);
  }

  public void hold(){
    m_motor.set(-.10);
  }
  public void stop(){
    m_motor.stopMotor();
  }

}
