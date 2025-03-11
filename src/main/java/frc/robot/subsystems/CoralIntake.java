package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralIntake extends SubsystemBase {
  private static final int deviceID = 0;//figure this out!!!!!
  private static TalonFX coralFunnelTalon;
  private static TalonFXConfiguration funnelTalonConfiguration;
  private static CurrentLimitsConfigs currentLimitConfigs;

  /** Creates a new CoralIntake. */
  public CoralIntake() {

    funnelTalonConfiguration = new TalonFXConfiguration();
    currentLimitConfigs = funnelTalonConfiguration.CurrentLimits;

    currentLimitConfigs.SupplyCurrentLowerLimit = 5;
    currentLimitConfigs.SupplyCurrentLimit = 40;
    currentLimitConfigs.SupplyCurrentLowerTime = 1.0;
    currentLimitConfigs.SupplyCurrentLimitEnable = true;

    currentLimitConfigs.StatorCurrentLimit = 80;
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    
    coralFunnelTalon = new TalonFX(deviceID);
    coralFunnelTalon.getConfigurator().apply(funnelTalonConfiguration);
    coralFunnelTalon.setNeutralMode(NeutralModeValue.Brake);
    coralFunnelTalon.setSafetyEnabled(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void prep(){
    coralFunnelTalon.set(0.01);
  }
  public void score(){
    coralFunnelTalon.set(0.5);
  }
  public void stop(){
    coralFunnelTalon.stopMotor();
  }
}
