// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new CameraSubsystem. */
  
  VideoSource usbCamera;

  public CameraSubsystem() {
    //usbCamera.setResolution(200, 100);
    //CameraServer.startAutomaticCapture(usbCamera);
    CameraServer.startAutomaticCapture("Rear Camera", 0);
  }

  public void initRearCamera(){
    CameraServer.startAutomaticCapture("Rear Camera", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
