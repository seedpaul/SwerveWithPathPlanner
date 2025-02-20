// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
//SparkMax motor controller
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeIntake extends SubsystemBase {

  private SparkMax RightElevMotor;
  private SparkMax LeftElevMotor;
  private SparkMaxConfig RightElevMotorConfig;
  private SparkMaxConfig LeftElevMotorConfig;
  private SparkClosedLoopController RightClosedLoopController;
  private SparkClosedLoopController LeftClosedLoopController;
  private RelativeEncoder RightEncoder;
  private RelativeEncoder LeftEncoder;

  /** Creates a new Elevator. */
  public AlgaeIntake() {
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    /************************************************************************************/
    //We need to add the actual device ids here!!!!
    /*************************************************************************************/
    RightElevMotor = new SparkMax(55, MotorType.kBrushless);
    LeftElevMotor = new SparkMax(9, MotorType.kBrushless);

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    RightClosedLoopController = RightElevMotor.getClosedLoopController();
    RightEncoder = RightElevMotor.getEncoder();

    LeftClosedLoopController = LeftElevMotor.getClosedLoopController();
    LeftEncoder = LeftElevMotor.getEncoder();

    RightElevMotorConfig = new SparkMaxConfig();
    LeftElevMotorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    RightElevMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    LeftElevMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    RightElevMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    LeftElevMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    RightElevMotor.configure(RightElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    LeftElevMotor.configure(LeftElevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
