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
  private SparkClosedLoopController RightClosedLoopController;
  private RelativeEncoder RightEncoder;

  private double targetPosition = 0;

  /** Creates a new Elevator. */
  public Elevator() {

    RightElevMotor = new SparkMax(19, MotorType.kBrushless);
    LeftElevMotor = new SparkMax(17, MotorType.kBrushless);

    RightClosedLoopController = RightElevMotor.getClosedLoopController();
    RightEncoder = RightElevMotor.getEncoder();

    RightElevMotorConfig = new SparkMaxConfig();
    LeftElevMotorConfig = new SparkMaxConfig();

    LeftElevMotorConfig.follow(RightElevMotor, true);  

    RightElevMotorConfig.voltageCompensation(12);
    LeftElevMotorConfig.voltageCompensation(12);

    RightElevMotorConfig.smartCurrentLimit(30,100,20);
    LeftElevMotorConfig.smartCurrentLimit(30,100,20);

    RightElevMotorConfig.inverted(true);
    LeftElevMotorConfig.inverted(true);
    
    RightElevMotorConfig.idleMode(IdleMode.kBrake);
    LeftElevMotorConfig.idleMode(IdleMode.kBrake);

    RightElevMotorConfig.closedLoopRampRate(4);
    LeftElevMotorConfig.closedLoopRampRate(4);

    RightEncoder.setPosition(0.0);
    
    RightElevMotorConfig.softLimit.reverseSoftLimit(0);
    RightElevMotorConfig.softLimit.reverseSoftLimitEnabled(true);

    RightElevMotorConfig.softLimit.forwardSoftLimit(40);
    RightElevMotorConfig.softLimit.forwardSoftLimitEnabled(true);

    RightElevMotorConfig.encoder.positionConversionFactor(1);
    RightElevMotorConfig.encoder.velocityConversionFactor(1);
    LeftElevMotorConfig.encoder.positionConversionFactor(1);
    LeftElevMotorConfig.encoder.velocityConversionFactor(1);

    RightElevMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.6)
        .i(0)
        .d(0.0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);  

    RightElevMotorConfig.closedLoop.maxMotion
      // Set MAXMotion parameters for position control. We don't need to pass
      // a closed loop slot, as it will default to slot 0.
      .maxVelocity(400)
      .maxAcceleration(700)
      .allowedClosedLoopError(1)
      // Set MAXMotion parameters for velocity control in slot 1
      .maxAcceleration(500, ClosedLoopSlot.kSlot1)
      .maxVelocity(6000, ClosedLoopSlot.kSlot1)
      .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

    RightElevMotor.configure(RightElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    LeftElevMotor.configure(LeftElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.putNumber("elevator Target Position", targetPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator Actual Position", RightEncoder.getPosition());
    SmartDashboard.putNumber("elevator Actual Velocity", RightEncoder.getVelocity());
    SmartDashboard.putNumber("elevator Target Position", targetPosition);

    //double targetPosition = SmartDashboard.getNumber("Target Position", 0);
    RightClosedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }
  public void up(){
    if(targetPosition <= 35){
      targetPosition +=5;
    }
    //RightElevMotor.set(0.2);
  }
  public void down(){
    if(targetPosition >= 5){
      targetPosition -=5;
    }
    //RightElevMotor.set(-0.1);
  }
  public void stop(){
    RightElevMotor.stopMotor();
  }
  public void hold(){
    RightElevMotor.set(0.05);
  }
}
