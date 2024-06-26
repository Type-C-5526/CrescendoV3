// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorBelt extends SubsystemBase {
  /** Creates a new ConveyorBelt. */

  private DigitalInput m_colorSensor;

  private TalonFX m_motor1;

  private static ConveyorBelt m_instance;
  private double m_motorVelocity = 0;
  public ConveyorBelt() {
    
    m_motor1 = new TalonFX(Constants.Conveyor.MotorID);


    m_colorSensor = new DigitalInput(Constants.Conveyor.ColorSensorChannel);

  }

  @Override
  public void periodic() {
    m_motorVelocity = m_motor1.get();
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Holding Note:", m_colorSensor.get());


  }
  public boolean hasGamePiece(){
    return m_colorSensor.get();
  }
  public boolean isReleasing(){
     if(m_motorVelocity < 0){
      return true;
    }
    else{
      return false;
    }

  }
  public boolean isHolding(){
     if(m_motorVelocity > 0){
      return true;
    }
    else{
      return false;
    }

  }

  public double getVelocity(){
    return m_motor1.getVelocity().getValueAsDouble();
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
