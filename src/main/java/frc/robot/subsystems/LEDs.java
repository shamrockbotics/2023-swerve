// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.ml.Ml;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;


  public static int[] green = {0, 255, 0};

  double m_rainbowFirstPixelHue = 0;

  public LEDs() {
    // PWM port 1 on Robo Rio
    // Must be PWM port, not MXP or DIO
    m_led = new AddressableLED(0);

    //Default length of 60, starts with empty output
    m_ledBuffer = new AddressableLEDBuffer(40);

    
    setUpLight();
  }

  public void setUpLight(){
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setAllRGBColor(int[] color){

    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      //Sets each LED to the same color

      if (i %2 == 0){
        m_ledBuffer.setRGB(i, (int)(color[0]*.1), (int)(color[1]*.1), (int)(color[2]*.1));
        //m_ledBuffer.setRGB(i, 0, 0, 0);
      } else {
        m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
      }
      
    }
    //updates every LED to their set color
    m_led.setData(m_ledBuffer);
  }

  public void setAllOff(){ // sets every LED to a color of 0 or off
    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void rainbow(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      final int hue = ((int)(m_rainbowFirstPixelHue) + (i * 180 / m_ledBuffer.getLength())) % 180;

      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
    // makes the rainbow move
    m_rainbowFirstPixelHue += 1;

    m_rainbowFirstPixelHue %= 180;


    m_led.setData(m_ledBuffer);
  }


  public void dimRainbow(double hueSpeed){
    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      final int hue = ((int)(m_rainbowFirstPixelHue) + (i * 180 / m_ledBuffer.getLength())) % 180;
      if (i %2 == 0){
        
        m_ledBuffer.setHSV(i, hue, 255, 40);
      } else {
        m_ledBuffer.setHSV(i, hue, 255, 255);
      }

      m_rainbowFirstPixelHue += hueSpeed;

      m_rainbowFirstPixelHue %= 180;

    }

    m_led.setData(m_ledBuffer);
  }

  public void cycleFlags(int[][][] flags, int flagNum){

    int[][] flag = flags[flagNum];

    int numLEDSPerColor = m_ledBuffer.getLength() / flag.length;

    int lastLEDSet = 0;

    for (int color = 0; color < flag.length; color++){
      //for every color in the flag
      for (int i = 0; i < numLEDSPerColor; i++){
        //for every led in the color
        //calculate what led needs set next
        int led = i + (numLEDSPerColor * color);
        m_ledBuffer.setRGB(led, flag[color][0], flag[color][1], flag[color][2]);
        lastLEDSet = led;
      }
    }
    if (m_ledBuffer.getLength() - lastLEDSet > 0){
      for (int i = lastLEDSet + 1; i < m_ledBuffer.getLength(); i++){
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
    }

    m_led.setData(m_ledBuffer);
  }







  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
