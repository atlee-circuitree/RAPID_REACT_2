// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import frc.robot.Constants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorSubsystem extends SubsystemBase {

  public final ColorSensorV3 colorSensor = new ColorSensorV3(Constants.i2cPort);

  public final static ColorMatch m_colorMatcher = new ColorMatch();

  public static Color Red = new Color(0.561, 0.232, 0.114);

  public static String sensorDashboard;
 
  public SensorSubsystem() {}

  @Override
  public void periodic() {
    
  }

  public static double isRed() {
    
    m_colorMatcher.addColorMatch(Red);

    if (m_colorMatcher.matchColor(Red) != null) {
    return 1;
    } else {
    return 0;
    }

  }
 
  public void startUSBCamera(){
    CameraServer.startAutomaticCapture();
  }



}
