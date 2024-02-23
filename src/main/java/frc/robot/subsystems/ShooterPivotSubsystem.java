// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivotSubsystem extends SubsystemBase {

  private static ShooterPivotSubsystem m_instance;

  private TalonFX m_motor;
  private CANcoder m_encoder;

  private final StatusSignal<Boolean> f_fusedSensorOutOfSync;
  private final StatusSignal<Boolean> sf_fusedSensorOutOfSync;
  private final StatusSignal<Boolean> f_remoteSensorOutOfSync;
  private final StatusSignal<Boolean> sf_remoteSensorOutOfSync;

  private final StatusSignal<Double> fx_pos;
  private final StatusSignal<Double> fx_vel;
  private final StatusSignal<Double> cc_pos;
  private final StatusSignal<Double> cc_vel;
  private final StatusSignal<Double> fx_rotorPos;


  /** Creates a new ShooterPivotSubsystem. */
  public ShooterPivotSubsystem() {
    m_motor = new TalonFX(18);
    m_encoder = new CANcoder(19);

    f_fusedSensorOutOfSync = m_motor.getFault_FusedSensorOutOfSync();
    sf_fusedSensorOutOfSync = m_motor.getStickyFault_FusedSensorOutOfSync();
    f_remoteSensorOutOfSync = m_motor.getFault_RemoteSensorDataInvalid();
    sf_remoteSensorOutOfSync = m_motor.getStickyFault_RemoteSensorDataInvalid();

    fx_pos = m_motor.getPosition();
    fx_vel = m_motor.getVelocity();
    cc_pos = m_encoder.getPosition();
    cc_vel  = m_encoder.getVelocity();
    fx_rotorPos = m_motor.getRotorPosition();

    CANcoderConfiguration m_CANcoderConfiguration = new CANcoderConfiguration();
    /* 
    m_CANcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    m_CANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_CANcoderConfiguration.MagnetSensor.MagnetOffset = 0.4;
    */
    m_CANcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    m_encoder.getConfigurator().apply(m_CANcoderConfiguration);

    TalonFXConfiguration m_FxConfiguration = new TalonFXConfiguration();
    m_FxConfiguration.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
    m_FxConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    m_motor.getConfigurator().apply(m_FxConfiguration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Cancoder Position", m_encoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Talon Position", m_motor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Pivot Degrees", getDegrees(m_encoder.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("Pivot Ticks", getEncoderTics(getDegrees(m_encoder.getPosition().getValueAsDouble())));

  }

  public double getEncoderTics(double _degrees){
    return (23291/8400000)*(_degrees * 42) + 0.266846;
  }
  public double getDegrees(double _encoderTicks){
    return (((_encoderTicks + 0.266846) * 8400000) / 23291) - 42;
  }
  public static ShooterPivotSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new ShooterPivotSubsystem();
    }
    return m_instance;
  }
}
