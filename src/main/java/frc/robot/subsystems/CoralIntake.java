package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralIntake extends SubsystemBase {
  private static final int deviceID = 0;//figure this out!!!!!
  private static TalonFX coralFunneTalon;
  private static TalonFXConfiguration funnelTalonConfiguration;
  private static CurrentLimitsConfigs currentLimitConfigs;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  /** Creates a new Climber. */
  public CoralIntake() {

    funnelTalonConfiguration = new TalonFXConfiguration();
    currentLimitConfigs = funnelTalonConfiguration.CurrentLimits;

    currentLimitConfigs.SupplyCurrentLowerLimit = 5;
    currentLimitConfigs.SupplyCurrentLimit = 40;
    currentLimitConfigs.SupplyCurrentLowerTime = 1.0;
    currentLimitConfigs.SupplyCurrentLimitEnable = true;

    currentLimitConfigs.StatorCurrentLimit = 80;
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    
    coralFunneTalon = new TalonFX(deviceID);
    coralFunneTalon.getConfigurator().apply(funnelTalonConfiguration);
    coralFunneTalon.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void prep(){
    coralFunneTalon.set(0.01);
  }
  public void score(){
    coralFunneTalon.set(0.5);
  }
  public void stop(){
    coralFunneTalon.stopMotor();
  }
}
