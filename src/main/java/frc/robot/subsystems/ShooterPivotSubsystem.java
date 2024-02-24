// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPivot;
import frc.robot.math.PhoenixUnits;

public class ShooterPivotSubsystem extends SubsystemBase {

  private static ShooterPivotSubsystem m_instance;

  private TalonFX m_motor;
  private CANcoder m_encoder;

  private boolean m_pidEnabled = false;
  private double m_setpoint = 0;


  /* Start at position 0, enable FOC, no feed forward, use slot 0 */
  private final PositionVoltage m_voltagePosition = new PositionVoltage(
    0, 
    0, 
    true, 
    0, 
    0, 
    false, 
    false, 
    false);

  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();


  /** Creates a new ShooterPivotSubsystem. */
  public ShooterPivotSubsystem() {
    m_motor = new TalonFX(ShooterPivot.MOTOR_ID, ShooterPivot.MOTOR_CANBUS);
    m_encoder = new CANcoder(ShooterPivot.CANCODER_ID, ShooterPivot.CANCODER_CANBUS);


    CANcoderConfiguration m_CANcoderConfiguration = new CANcoderConfiguration();
    
    m_CANcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    m_CANcoderConfiguration.MagnetSensor.MagnetOffset = -0.766846;
    m_CANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    
    m_encoder.getConfigurator().apply(m_CANcoderConfiguration);

    //m_encoder.setPosition(m_encoder.getAbsolutePosition().getValueAsDouble() - 0.382080);

    TalonFXConfiguration m_FxConfiguration = new TalonFXConfiguration();
    m_FxConfiguration.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
    m_FxConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    m_FxConfiguration.Slot0.kP = ShooterPivot.PIVOT_PID_UTIL.getP(); // An error of 0.5 rotations results in 1.2 volts output
    m_FxConfiguration.Slot0.kD = ShooterPivot.PIVOT_PID_UTIL.getD(); // 0.1  A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    m_FxConfiguration.Voltage.PeakForwardVoltage = 12;
    m_FxConfiguration.Voltage.PeakReverseVoltage = -12;

    m_FxConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motor.getConfigurator().apply(m_FxConfiguration);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.

    var encoderPosition = m_encoder.getPosition().getValueAsDouble();
    var motorPosition = m_motor.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("Pivot Cancoder Position", encoderPosition);
    SmartDashboard.putNumber("Pivot Talon Position", motorPosition);

    SmartDashboard.putBoolean("Pivot Enabled", m_pidEnabled);
    SmartDashboard.putBoolean("Pivot At Setpoint", atSetpoint());

    if (m_pidEnabled) {
      m_motor.setControl(m_voltagePosition.withPosition(PhoenixUnits.getDegreesToRotations(m_setpoint)));
    }else{
      m_motor.setControl(m_brake);
    }
  }

  public void enablePID(){
    m_pidEnabled = true;
  }

  public void disablePID(){
    m_pidEnabled = false;
  }

  public void setSetpointInDegrees(double _setpoint){
    m_setpoint = PhoenixUnits.getDegreesToRotations(_setpoint);
  }

  public boolean atSetpoint(){
    return ShooterPivot.PIVOT_PID_UTIL.atSetpoint(m_encoder.getPosition().getValueAsDouble(), m_setpoint);
  }

  public void resetPositionToAbsolute(){
    m_encoder.setPosition(m_encoder.getAbsolutePosition().getValueAsDouble());
  }

  public static ShooterPivotSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new ShooterPivotSubsystem();
    }
    return m_instance;
  }
}