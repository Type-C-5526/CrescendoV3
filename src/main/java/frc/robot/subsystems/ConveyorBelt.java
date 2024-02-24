// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorBelt extends SubsystemBase {
  /** Creates a new ConveyorBelt. */
  private boolean m_enabled;

  private CANSparkMax m_motor1;
  private SparkPIDController m_pidController;

  private static ConveyorBelt m_instance;
  private double m_MotorOutput;
  private double m_setpoint;
  public ConveyorBelt() {
    m_enabled = false;

    m_motor1 = new CANSparkMax(Constants.Conveyor.MotorID, MotorType.kBrushless);
    m_motor1.restoreFactoryDefaults();

    m_pidController = m_motor1.getPIDController();

    m_pidController.setP(Constants.Conveyor.ConveyorPIDConstants.getP());
    m_pidController.setI(Constants.Conveyor.ConveyorPIDConstants.getI());
    m_pidController.setD(Constants.Conveyor.ConveyorPIDConstants.getD());
    m_pidController.setOutputRange(-1, 1);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if(m_enabled){
    //m_pidController.setReference(m_setpoint, CANSparkMax.ControlType.kPosition);
    }
    else {
      //m_motor1.stopMotor();
    }


  }

  public void setSetpoint(double _Setpoint){
    m_setpoint = _Setpoint;
  }

  public void enableMotorPID(){
    m_enabled = true;
  }

  public void disableMotorPID(){
    m_enabled = false;
  }



  public void setMotorVelocity(double _velocity){
    m_motor1.set(_velocity);
  }

  public static ConveyorBelt getInstance(){
    
    if(m_instance == null){
      m_instance = new ConveyorBelt();

    }
    return m_instance;

  }
}
