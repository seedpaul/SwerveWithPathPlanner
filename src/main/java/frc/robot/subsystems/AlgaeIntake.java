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

  private SparkMax RightElevMotor;
  private SparkMax LeftElevMotor;
  private SparkMaxConfig RightElevMotorConfig;
  private SparkMaxConfig LeftElevMotorConfig;

  /** Creates a new Elevator. */
  public AlgaeIntake() {

    RightElevMotor = new SparkMax(55, MotorType.kBrushless);
    LeftElevMotor = new SparkMax(9, MotorType.kBrushless);

    RightElevMotorConfig = new SparkMaxConfig();
    LeftElevMotorConfig = new SparkMaxConfig();

    RightElevMotor.configure(RightElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    LeftElevMotor.configure(LeftElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void in(){
    RightElevMotor.set(0.1);
    LeftElevMotor.set(-0.1);
  }
  public void out(){
    RightElevMotor.set(-0.1);
    LeftElevMotor.set(0.1);
  }
  public void stop(){
    RightElevMotor.stopMotor();
    LeftElevMotor.stopMotor();
  }
}
