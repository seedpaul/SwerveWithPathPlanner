package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator_minion extends SubsystemBase {

  private final TalonFXS fxs_Right;
  private final TalonFXS fxs_Left;
  private final TalonFXSConfiguration right_cfg;
  private final TalonFXSConfiguration left_cfg;

  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  //private RelativeEncoder RightAlternateEncoder;
  private int[] setpoints = { 0, 10, 20, 30, 40, 50 };
  private int currentSetpoint = 0;
  private int targetPosition = 0;

  /** Creates a new Elevator. */
  public Elevator_minion() {

    fxs_Right = new TalonFXS(7, "CANivore2");
    fxs_Left = new TalonFXS(27, "CANivore2");
    right_cfg = new TalonFXSConfiguration();
    left_cfg = new TalonFXSConfiguration();

    rightOut.UpdateFreqHz = 0;

    right_cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    left_cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    right_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    left_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    right_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.35;
    left_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.35;

    right_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    right_cfg.CurrentLimits.StatorCurrentLimit = 20;
    right_cfg.CurrentLimits.StatorCurrentLimitEnable = false; // Start with stator limits off
    right_cfg.CurrentLimits.SupplyCurrentLowerLimit = 5;
    right_cfg.CurrentLimits.SupplyCurrentLimit = 40;
    right_cfg.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    right_cfg.CurrentLimits.SupplyCurrentLimitEnable = false; // Start with supply limits off

    left_cfg.CurrentLimits.StatorCurrentLimit = 20;
    left_cfg.CurrentLimits.StatorCurrentLimitEnable = false; // Start with stator limits off
    left_cfg.CurrentLimits.SupplyCurrentLowerLimit = 5;
    left_cfg.CurrentLimits.SupplyCurrentLimit = 40;
    left_cfg.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    left_cfg.CurrentLimits.SupplyCurrentLimitEnable = false; // Start with supply limits off

    fxs_Right.getConfigurator().apply(right_cfg);
    fxs_Left.getConfigurator().apply(left_cfg);

    // Apply the follower request to the follower Talon
    //fxs_Left.setControl(new Follower(fxs_Right.getDeviceID(), true));

    SmartDashboard.putNumber("elevator Target Position", targetPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("elevator Primary Position", RightRelativeEncoder.getPosition());
    //SmartDashboard.putNumber("elevator Absolute Position", RightAlternateEncoder.getPosition());
    //SmartDashboard.putNumber("elevator Velocity", RightRelativeEncoder.getVelocity());
    //SmartDashboard.putNumber("elevator Target Position", setpoints[currentSetpoint]);
  }

  public void up() {
    if (currentSetpoint < setpoints.length) {
      currentSetpoint++;
    }
  }

  public void down() {
    if (currentSetpoint > 0) {
      currentSetpoint--;
    }
  }

  public void stop() {
    fxs_Right.stopMotor();
  }

  public void hold() {
    fxs_Right.setControl(rightOut.withOutput(0.0));
  }

  public void upManual() {
    fxs_Right.setControl(rightOut.withOutput(1));
  }

  public void downManual() {
    fxs_Right.setControl(rightOut.withOutput(-.4));
  }
}
