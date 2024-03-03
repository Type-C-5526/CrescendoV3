// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDs;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED m_leds;
  private AddressableLEDBuffer m_ledsbuffer;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {

    m_leds = new AddressableLED(LEDs.LedPort);
    m_ledsbuffer = new AddressableLEDBuffer(LEDs.LedLength);

    m_leds.setLength(m_ledsbuffer.getLength());

    m_leds.setData(m_ledsbuffer);

    m_leds.start();



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_leds.setData(m_ledsbuffer);

    switch (Superstructure.getRobotStatus()){
      case AIMED:
        
        break;
      case AIMING:

        break;
    }
      
  }

  public void solid(){

  }
  public void Ascend(Color color) {
  
    }

  public void blink(){

  }
  public void epilepticAttack(){

  }

  public void fire(){

  }
  public void typeCMorseCode(){

  }

}
