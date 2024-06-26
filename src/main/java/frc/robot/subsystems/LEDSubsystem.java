// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDs;
import frc.robot.subsystems.Superstructure.RobotStatus;
import frc.robot.util.LimelightHelpers;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED m_leds;
  private AddressableLEDBuffer m_ledsbuffer;
  private boolean m_hasToRepeat = false;
  private int m_reapeatCounter = 0;
  private int m_timesToRepeat = 0;
  private RobotStatus m_StatusToRepeat;
  private boolean m_hasAppliedAllianceColor;

  Random random = new Random();

  private Timer m_timer;

  private static LEDSubsystem m_instance;;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {

    m_timer = new Timer();
    m_timer.reset();
    m_timer.start();

    m_leds = new AddressableLED(LEDs.LedPort);
    m_ledsbuffer = new AddressableLEDBuffer(LEDs.LedLength);

    m_leds.setLength(m_ledsbuffer.getLength());

    m_leds.setData(m_ledsbuffer);

    m_leds.start();



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_hasAppliedAllianceColor = false;

    if (DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent((allianceColor) -> {

          if(allianceColor == Alliance.Red){
            blink(Color.kRed);
          }else if(allianceColor == Alliance.Blue){
            
            blink(Color.kBlue);
          }

          m_leds.setData(m_ledsbuffer);
          m_hasAppliedAllianceColor = true;
      });

      if(m_hasAppliedAllianceColor){
        return;
      }else{
        ascendingAnimation(Color.kAliceBlue);
       //blink(Color.kYellow);
        m_leds.setData(m_ledsbuffer);
        return;
      }
    }

    if(m_hasToRepeat && m_reapeatCounter < m_timesToRepeat){
      m_reapeatCounter++;
      assignLedStatus(m_StatusToRepeat);

      VisionSubsystem.getInstance().getAprilTagCamera().blinkLEDS();
    }
    else {
      m_hasToRepeat = false;
      m_reapeatCounter = 0;
      m_timesToRepeat = 0;
      assignLedStatus(Superstructure.getRobotStatus());

      VisionSubsystem.getInstance().getAprilTagCamera().turnOffLEDS();
    }

   

     m_leds.setData(m_ledsbuffer);
      
  }

  public void solid(Color color){
    for (int i = 0; i < m_ledsbuffer.getLength(); i++) {  
      m_ledsbuffer.setLED(i, color);
    }
  }

    public void ascendingAnimation(Color color) {
    double elapsedTime = m_timer.get();
    double duration = 0.8; // Total duration of the animation

    // Calculate the ratio of elapsed time to duration
    double ratio = (elapsedTime % duration) / duration;

    // Calculate the number of LEDs to light up based on the ratio
    int numLedsLit = (int) (ratio * m_ledsbuffer.getLength());

    // Set LEDs to the specified color up to numLedsLit
    for (int i = 0; i < numLedsLit; i++) {
        m_ledsbuffer.setLED(i, color);
    }

    // Set the rest of the LEDs to black
    for (int i = numLedsLit; i < m_ledsbuffer.getLength(); i++) {
        m_ledsbuffer.setLED(i, Color.kBlack);
    }

    if (elapsedTime > duration) {
        m_timer.reset();
        m_timer.start();
    }
  }




  public void blink(Color color){
    if(m_timer.get() < 0.1){
      for (int i = 0; i < m_ledsbuffer.getLength(); i++) {  
        m_ledsbuffer.setLED(i, color);
      }
    }
    else if(m_timer.get() > 0.5){
      for (int i = 0; i < m_ledsbuffer.getLength(); i++) {  
        m_ledsbuffer.setLED(i, Color.kBlack);
      }
    }

    if(m_timer.get() > 1){
      m_timer.reset();
      m_timer.start();
    }

  }

  public void epilepticAttack(Color color){
    if(m_timer.get() < 0.025){
      for (int i = 0; i < m_ledsbuffer.getLength(); i++) {  
        m_ledsbuffer.setLED(i, color);
      }
    }
    else if(m_timer.get() > 0.05){
      for (int i = 0; i < m_ledsbuffer.getLength(); i++) {  
        m_ledsbuffer.setLED(i, Color.kBlack);
      }
    }

    if(m_timer.get() > 0.1){
      m_timer.reset();
      m_timer.start();
    }
  }

 
  public void typeCMorseCode(Color color){
  

  }
  public void fireSlime() {
    double currentTime = m_timer.get();
    double period = 0.2; // Adjust as needed for slower animation

    // Define base color components for fire (red to orange gradient)
    int baseRed = 255;
    int baseGreen = 69; // Adjust as needed
    int baseBlue = 0;

    // Calculate flicker intensity
    double flickerIntensity = Math.sin(currentTime * Math.PI / period);

    // Set LEDs colors based on the fire effect
    for (int i = 0; i < m_ledsbuffer.getLength(); i++) {
        // Calculate intensity for the current LED
        double intensityFactor = (double) (i) / (m_ledsbuffer.getLength() - 1); // Higher intensity at the bottom
        double flicker = flickerIntensity * intensityFactor;
        
        // Apply flicker effect
        int flickerRed = (int) (baseRed * (1 - flicker));
        int flickerGreen = (int) (baseGreen * (1 - flicker));
        int flickerBlue = (int) (baseBlue * (1 - flicker));

        // Apply gradient to fade to orange from opposite ends
        int finalRed = (int) ((flickerRed * intensityFactor) + ((255 - flickerRed) * (1 - intensityFactor)));
        int finalGreen = (int) ((flickerGreen * intensityFactor) + ((165 - flickerGreen) * (1 - intensityFactor)));
        int finalBlue = (int) ((flickerBlue * intensityFactor) + ((0 - flickerBlue) * (1 - intensityFactor)));
        
        // Set the LED color
        m_ledsbuffer.setLED(i, new Color(finalRed, finalGreen, finalBlue));
    }
}


  public static LEDSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new LEDSubsystem();
    }
    return m_instance;
     
  }


  void setPixelHeatColor(int pixel, byte temperature) {
    // Rescale heat from 0-255 to 0-191
    byte t192 = (byte) Math.round((temperature / 255.0) * 191);

    // Calculate ramp up from
    byte heatRamp = (byte) (t192 & 0x3F); // 0...63
    heatRamp <<= 2; // scale up to 0...252

    // Figure out which third of the spectrum we're in:
    if (t192 > 0x80) { // hottest
        // Adjust these values to match your FRC LED color scheme
        m_ledsbuffer.setLED(pixel, new Color(255, 255, heatRamp));
    } else if (t192 > 0x40) { // middle
        m_ledsbuffer.setLED(pixel, new Color(255, heatRamp, 0));
    } else { // coolest
        m_ledsbuffer.setLED(pixel, new Color(heatRamp, 0, 0));
    }
  }

  public void assignLedStatus(RobotStatus _status){
     switch (_status){
      case AIMED:
        epilepticAttack(Color.kYellow);
        break;
      case AIMING:
      solid(Color.kYellow);

        break;
      case HOME:
        solid(Color.kPurple);
        break;

      case LEAVING_IN_AMP:
      ascendingAnimation(Color.kOrange);

        break;

      case ELEVATING:

        break;

      case PICKING_FROM_FLOOR:
        solid(Color.kGreen);
        break;

      case PICKING_FROM_SOURCE:
        solid(Color.kGreen);
        break;

      case SCORING_IN_AMP:
        epilepticAttack(Color.kOrange);
        break;

      case SHOOTING:
      
        break;
      case HAS_GAME_PIECE:
        m_hasToRepeat = true;
        m_timesToRepeat = 100;
        m_StatusToRepeat = RobotStatus.HAS_GAME_PIECE;
        epilepticAttack(Color.kGreen);
        break;
      case CANT_AIM:
        m_hasToRepeat = true;
        m_timesToRepeat = 60;
        m_StatusToRepeat = RobotStatus.CANT_AIM;
        epilepticAttack(Color.kRed);
        break;
      default:

        break;
    }
  }

}
