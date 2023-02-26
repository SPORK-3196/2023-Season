// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Lighting extends SubsystemBase {

public static AddressableLED lightsAlpha = new AddressableLED(0);

public static AddressableLEDBuffer lightBufferAlpha;

public static int firstPixelHue = 0;

double alphaLightCounter;
boolean alphaLightUp = true;

public Lighting(){
    lightBufferAlpha = new AddressableLEDBuffer(300);

    alphaLightCounter = lightBufferAlpha.getLength();

    lightsAlpha.setLength(lightBufferAlpha.getLength());

    lightsAlpha.setData(lightBufferAlpha);

}

public void start(){
    lightsAlpha.start();
}

public void noColor(){
    for(int i= 0; i < lightBufferAlpha.getLength(); i++){
        lightBufferAlpha.setRGB(i, 0, 0, 0);
    }
    lightsAlpha.setData(lightBufferAlpha);
}

public void fullRed(){
    for(int i= 0; i < lightBufferAlpha.getLength(); i++){
        lightBufferAlpha.setRGB(i, 255, 0, 0);
    }
    lightsAlpha.setData(lightBufferAlpha);
}

public void fullGreen(){
    for(int i= 0; i < lightBufferAlpha.getLength(); i++){
        lightBufferAlpha.setRGB(i, 0, 255, 0);
    }
    lightsAlpha.setData(lightBufferAlpha);
}

public void fullBlue(){
    for(int i= 0; i < lightBufferAlpha.getLength(); i++){
        lightBufferAlpha.setRGB(i, 0, 0, 255);
    }
    lightsAlpha.setData(lightBufferAlpha);
}
//rainbow time come on with your frends  
public void FullRainbow() {
    for (int i = 0; i < lightBufferAlpha.getLength(); i++) {
      final int hue = (firstPixelHue + (i * 180 / lightBufferAlpha.getLength())) % 180;
      lightBufferAlpha.setHSV(i, hue, 169, 35);
    }
    firstPixelHue += 3;

    firstPixelHue %= 180;
    lightsAlpha.setData(lightBufferAlpha);

  }

  /** Set lights to rainbow, constantly changing */
  public void rainbowRun() {
    for (int i = 0; i < lightBufferAlpha.getLength() - alphaLightCounter; i++) {
        final int hue = (firstPixelHue + (i * 180 / lightBufferAlpha.getLength())) % 180;
        lightBufferAlpha.setHSV(i, hue, 169, 100);
    }

    lightsAlpha.setData(lightBufferAlpha);


    if (alphaLightCounter == 0) {
      alphaLightCounter = lightBufferAlpha.getLength();
    }
    

    
    firstPixelHue += 3;
    firstPixelHue %= 180;
    alphaLightCounter -= 10;
  }


}
