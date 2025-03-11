package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Climber extends SubsystemBase {
  private static final int deviceID = 44;
  private SparkFlex m_motor;
  private RelativeEncoder m_encoder;
  private SparkMaxConfig m_motorConfig;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  /** Creates a new Climber. */
  public Climber() {

    m_motor = new SparkFlex(deviceID, MotorType.kBrushless);
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
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climber posistion:",m_encoder.getPosition());
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
