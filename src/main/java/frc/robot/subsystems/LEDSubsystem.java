// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDs;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED m_leds;
  private AddressableLEDBuffer m_ledsbuffer;

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
   
    switch (Superstructure.getRobotStatus()){
      case AIMED:
        fire(500,5000,50);
        break;
      case AIMING:

        break;
      case HOME:


        break;

      case ALIGNING_TO_AMP:

        break;

      case ELEVATING:

        break;

      case PICKING_FROM_FLOOR:

        break;

      case PICKING_FROM_SOURCE:

        break;

      case SCORING_IN_AMP:

        break;

      case SHOOTING:

        break;
      default:

        break;
    }

     m_leds.setData(m_ledsbuffer);
      
  }

  public void solid(Color color){
    for (int i = 0; i < m_ledsbuffer.getLength(); i++) {  
      m_ledsbuffer.setLED(i, color);
    }
  }
  public void Ascend(Color color) {
  
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
    if(m_timer.get() < 0.05){
      for (int i = 0; i < m_ledsbuffer.getLength(); i++) {  
        m_ledsbuffer.setLED(i, color);
      }
    }
    else if(m_timer.get() > 0.1){
      for (int i = 0; i < m_ledsbuffer.getLength(); i++) {  
        m_ledsbuffer.setLED(i, Color.kBlack);
      }
    }

    if(m_timer.get() > 0.2){
      m_timer.reset();
      m_timer.start();
    }
  }

  public void fire(int flameHeight, int sparks, int delayDuration){
    byte[] heat = new byte[m_ledsbuffer.getLength()];
    int cooldown;

    // Enfría cada celda un poco
    for (int i = 0; i < m_ledsbuffer.getLength(); i++) {
        cooldown = random.nextInt(((m_ledsbuffer.getLength() * 10) / m_ledsbuffer.getLength()) + 2);

        if (cooldown > heat[i]) {
            heat[i] = 0;
        } else {
            heat[i] -= cooldown;
        }
    }

    // El calor de cada celda se desplaza hacia arriba y difunde ligeramente
    for (int k = (m_ledsbuffer.getLength() - 1); k >= 2; k--) {
        heat[k] = (byte) ((heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3);
    }

    // Enciende aleatoriamente nuevas chispas cerca de la base de la llama
    if (random.nextInt(255) < sparks) {
        int y = random.nextInt(7);
        heat[y] += random.nextInt(160) + 160;
    }

    // Convierte el calor en colores LED
    for (int j = 0; j < m_ledsbuffer.getLength(); j++) {
        setPixelHeatColor(j, heat[j]);
    }

    // Muestra los LEDs
    // Necesitarás adaptar esta parte para que funcione con el sistema de LED de tu robot FRC
    // Por ejemplo, si estás usando WPILib, establecerías los LEDs así:
    // m_ledsbuffer.setLED(i, new Color(r, g, b));
    // m_ledsbuffer.show();
    
    // Retardo
    try {
        Thread.sleep(delayDuration);
    } catch (InterruptedException e) {
        e.printStackTrace();
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

}
