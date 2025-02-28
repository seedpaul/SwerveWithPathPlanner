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

    RightElevMotorConfig.smartCurrentLimit(30,60,200);
    LeftElevMotorConfig.smartCurrentLimit(30,60,200);

    RightElevMotorConfig.inverted(true);
    LeftElevMotorConfig.inverted(true);
    
    RightElevMotorConfig.idleMode(IdleMode.kBrake);
    LeftElevMotorConfig.idleMode(IdleMode.kBrake);

    RightEncoder.setPosition(0.0);
    
    // RightElevMotorConfig.softLimit.reverseSoftLimit(-45);
    // RightElevMotorConfig.softLimit.reverseSoftLimitEnabled(true);

    // RightElevMotorConfig.softLimit.forwardSoftLimit(1);
    // RightElevMotorConfig.softLimit.forwardSoftLimitEnabled(true);

    RightElevMotorConfig.encoder
        .positionConversionFactor(.1)
        .velocityConversionFactor(.1);

    LeftElevMotorConfig.encoder
        .positionConversionFactor(.1)
        .velocityConversionFactor(.1);

    // RightElevMotorConfig.closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     // Set PID values for position control. We don't need to pass a closed loop
    //     // slot, as it will default to slot 0.
    //     .p(0.1)
    //     .i(0)
    //     .d(0)
    //     .outputRange(-1, 1)
    //     // Set PID values for velocity control in slot 1
    //     .p(0.0001, ClosedLoopSlot.kSlot1)
    //     .i(0, ClosedLoopSlot.kSlot1)
    //     .d(0, ClosedLoopSlot.kSlot1)
    //     .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1);  

    RightElevMotor.configure(RightElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    LeftElevMotor.configure(LeftElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", RightEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", RightEncoder.getVelocity());

    if (SmartDashboard.getBoolean("Reset Encoder", false)) {
      SmartDashboard.putBoolean("Reset Encoder", false);
      // Reset the encoder position to 0
      RightEncoder.setPosition(0);
    }

    double targetPosition = SmartDashboard.getNumber("Target Position", 0);
    //RightClosedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  public void up(){
    RightElevMotor.set(0.3);
  }
  public void down(){
    RightElevMotor.set(-0.1);
  }
  public void stop(){
    RightElevMotor.stopMotor();
  }
  public void hold(){
    RightElevMotor.set(0.05);
  }
}
