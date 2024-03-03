// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private TalonFX m_motor1;
  private TalonFX m_followerMotor;

  private TalonFXConfiguration m_motor1Configuration;

  private PositionVoltage m_PositionVoltage;
  private double m_setpoint;

  private boolean m_enabled;

  private NeutralOut m_break;

  private static ElevatorSubsystem m_instance;
  


  public ElevatorSubsystem() {

    m_PositionVoltage = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
    
    m_setpoint = 0;
    m_enabled = false;
    m_break = new NeutralOut();
    m_motor1 = new TalonFX(Constants.Elevator.talonFXElevatorID, Constants.Elevator.canbus);
    m_followerMotor = new TalonFX(Constants.Elevator.talonFollowerFXElevatorID, Constants.Elevator.canbus);

    m_motor1Configuration = new TalonFXConfiguration();

    m_motor1Configuration.Slot0.kP = Constants.Elevator.pidConstants.getP();
    m_motor1Configuration.Slot0.kI = Constants.Elevator.pidConstants.getI();
    m_motor1Configuration.Slot0.kD = Constants.Elevator.pidConstants.getD();

    m_motor1Configuration.Voltage.PeakForwardVoltage = 8;
    m_motor1Configuration.Voltage.PeakReverseVoltage = -8;

    m_motor1Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motor1Configuration.CurrentLimits.StatorCurrentLimit = 150;
    m_motor1Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor1Configuration.CurrentLimits.SupplyCurrentLimit = 40;
    
    m_motor1.getConfigurator().apply(m_motor1Configuration);

    m_followerMotor.getConfigurator().apply(m_motor1Configuration);

    m_followerMotor.setControl(new Follower(m_motor1.getDeviceID(), true));
    m_motor1.setPosition(0);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_enabled){
      m_motor1.setControl(m_PositionVoltage.withPosition(m_setpoint));
    }
    else {
      m_motor1.setControl(m_break);
    }
    SmartDashboard.putNumber("Elevator encoder motor", m_motor1.getPosition().getValueAsDouble());

  }

  public void setSetpointAsPercent(double _Setpoint){
    m_setpoint = _Setpoint * -0.0977055;
  }

  public void enableMotorPID() {
    m_enabled = true;
  }

  public void disableMotorPID() {
    m_enabled = false;
  }




  public static ElevatorSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new ElevatorSubsystem();
    }
    return m_instance;
  }

}
