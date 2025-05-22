package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import static edu.wpi.first.units.Units.*;

public class Elevator_minion extends SubsystemBase {

  private final TalonFXS fxs_Right;
  private final TalonFXS fxs_Left;
  private final TalonFXSConfiguration right_cfg;
  private final TalonFXSConfiguration left_cfg;
  private final PositionVoltage pid_request = new PositionVoltage(0).withSlot(0);
  final MotionMagicVoltage mm_request = new MotionMagicVoltage(0).withSlot(0);
  private final Mechanisms m_mechanism = new Mechanisms();
  private final NeutralOut m_brake = new NeutralOut();

  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  private double[] setpointsCoral = { 0, 12, 27, 48.75};
  private int currentSetpointIndex = 0;
  private double targetPosition = 0;

  /** Creates a new Elevator. */
  public Elevator_minion() {

    fxs_Right = new TalonFXS(7, "CANivore2");
    fxs_Left = new TalonFXS(27, "CANivore2");
    right_cfg = new TalonFXSConfiguration();
    left_cfg = new TalonFXSConfiguration();

    rightOut.UpdateFreqHz = 0;

    fxs_Right.setSafetyEnabled(false);
    fxs_Left.setSafetyEnabled(false);

    right_cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    left_cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    right_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    left_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    right_cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

    right_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    right_cfg.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    right_cfg.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    right_cfg.SoftwareLimitSwitch.withForwardSoftLimitThreshold(64);
    right_cfg.SoftwareLimitSwitch.withReverseSoftLimitThreshold(0);

    right_cfg.CurrentLimits.StatorCurrentLimit = 120;
    right_cfg.CurrentLimits.StatorCurrentLimitEnable = false;
    right_cfg.CurrentLimits.SupplyCurrentLowerLimit = 5;
    right_cfg.CurrentLimits.SupplyCurrentLimit = 80;
    right_cfg.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    right_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    left_cfg.CurrentLimits.StatorCurrentLimit = 120;
    left_cfg.CurrentLimits.StatorCurrentLimitEnable = false;
    left_cfg.CurrentLimits.SupplyCurrentLowerLimit = 5;
    left_cfg.CurrentLimits.SupplyCurrentLimit = 80;
    left_cfg.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    left_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    fxs_Right.setSafetyEnabled(false);
    fxs_Left.setSafetyEnabled(false);

    right_cfg.Slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    right_cfg.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    right_cfg.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    // right_cfg.Slot0.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    // right_cfg.Slot0.kI = 0; // no output for integrated error
    // right_cfg.Slot0.kD = 0.1; // A velocity error of 1 rps re

    right_cfg.MotionMagic.MotionMagicCruiseVelocity = 65; // Target cruise velocity of 80 rps
    right_cfg.MotionMagic.MotionMagicAcceleration = 140; // Target acceleration of 160 rps/s (0.5 seconds)
    right_cfg.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    //sl0t 0 gains are for upward movement
    right_cfg.Slot0.kP = 1.15; 
    right_cfg.Slot0.kI = 0.0; 
    right_cfg.Slot0.kD = 0.3; 
    // right_cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    // right_cfg.Slot0.kG = 1.0;
    // right_cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    // right_cfg.Slot0.kS = 0.1;

    //slot 1 gain are for downward movement
    //right_cfg.Slot1.kP = .3; 
    //right_cfg.Slot1.kI = 0.0;
    //right_cfg.Slot1.kD = 0.0;

    right_cfg.Voltage.withPeakForwardVoltage(Volts.of(16))
      .withPeakReverseVoltage(Volts.of(-16));

    this.setConfigs();

    // Apply the follower request to the er Talon
    fxs_Left.setControl(new Follower(fxs_Right.getDeviceID(), true));
    SmartDashboard.putNumber("elevator Target Position", targetPosition);

    /* Make sure we start at 0 */
    fxs_Right.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_mechanism.update(fxs_Right.getPosition());

    var rotorPosSignal = fxs_Right.getRotorPosition();
    SmartDashboard.putNumber("elevator Primary Position", rotorPosSignal.getValueAsDouble());
    SmartDashboard.putNumber("elevator target Position", targetPosition);
    SmartDashboard.putNumber("elevator closed loop error", fxs_Right.getClosedLoopError().getValue());

    this.displayPosition();
    this.getPositionFromDash();

  }

  public void up() {
    if (currentSetpointIndex < (setpointsCoral.length-1)) {
      currentSetpointIndex++;
      targetPosition = setpointsCoral[currentSetpointIndex];
      //move to target position using upward movement gains
      fxs_Right.setControl(mm_request.withPosition(targetPosition).withSlot(0));
    }
  }

  public void down() {
    if (currentSetpointIndex > 0) {
      currentSetpointIndex--;
      targetPosition = setpointsCoral[currentSetpointIndex];
      //move to target position using downward movement gains
      fxs_Right.setControl(mm_request.withPosition(targetPosition).withSlot(0));
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
    fxs_Right.setControl(rightOut.withOutput(-.3));
  }

  private void setConfigs(){
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
          status = fxs_Right.getConfigurator().apply(right_cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not apply right configs, error code: " + status.toString());
        }
    
        for (int i = 0; i < 5; ++i) {
          status = fxs_Left.getConfigurator().apply(left_cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not left apply configs, error code: " + status.toString());
        }

  }

  private void displayPosition(){

    boolean[] currentPosition = {false,false,false,false};
    currentPosition[currentSetpointIndex] = true;

    SmartDashboard.putBoolean("Trough", currentPosition[0]);
    SmartDashboard.putBoolean("L-2", currentPosition[1]);
    SmartDashboard.putBoolean("L-3", currentPosition[2]);
    SmartDashboard.putBoolean("L-4", currentPosition[3]);

    SmartDashboard.putNumber("Trough setpoint", setpointsCoral[0]);
    SmartDashboard.putNumber("L-2 setpoint", setpointsCoral[1]);
    SmartDashboard.putNumber("L-3 setpoint", setpointsCoral[2]);
    SmartDashboard.putNumber("L-4 setpoint", setpointsCoral[3]);

  }

  private void getPositionFromDash(){

    double trough = SmartDashboard.getNumber("Trough", setpointsCoral[0]);
    double l2 = SmartDashboard.getNumber("L-2", setpointsCoral[1]);
    double l3 = SmartDashboard.getNumber("L-3", setpointsCoral[2]);
    double l4 = SmartDashboard.getNumber("L-4", setpointsCoral[3]);

    setpointsCoral[0] = (int)trough;
    setpointsCoral[1] = (int)l2;
    setpointsCoral[2] = (int)l3;
    setpointsCoral[3] = (int)l4;

  }

  public int getCoralLevel(){
    //return current level
    // 1 = trough,2 = L-2,3 = L-3,4 = L-4
    return currentSetpointIndex + 1;
  }

  public void auto_gotoLevel(int level){

    currentSetpointIndex = level-1;
    targetPosition = setpointsCoral[currentSetpointIndex];
    fxs_Right.setControl(mm_request.withPosition(targetPosition).withSlot(0));

  }

}
