// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//SparkMax motor controller
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeIntake extends SubsystemBase {

  private SparkMax RightAIMotor;
  private SparkMax LeftAIMotor;
  private SparkMaxConfig RightAIMotorConfig;
  private SparkMaxConfig LeftAIMotorConfig;

  /** Creates a new Elevator. */
  public AlgaeIntake() {

    RightAIMotor = new SparkMax(55, MotorType.kBrushless);
    LeftAIMotor = new SparkMax(9, MotorType.kBrushless);

    RightAIMotorConfig = new SparkMaxConfig();
    LeftAIMotorConfig = new SparkMaxConfig();

    RightAIMotor.configure(RightAIMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    LeftAIMotor.configure(LeftAIMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void in(){
    RightAIMotor.set(-0.75);
    LeftAIMotor.set(0.75);
  }
  public void out(){
    RightAIMotor.set(0.75);
    LeftAIMotor.set(-0.75);
  }
  public void stop(){
    RightAIMotor.stopMotor();
    LeftAIMotor.stopMotor();
  }
  public void hold(){
    RightAIMotor.set(-0.1);
    LeftAIMotor.set(0.1);
  }
}
