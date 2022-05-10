// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  CANSparkMax feederMotor = null;

  public FeederSubsystem() {

    feederMotor = new CANSparkMax(Constants.feederMotorPort, MotorType.kBrushless);
    
  }

  public void runFeeder(double speed) {

    feederMotor.set(speed);

  }
 
  public void startUSBCamera(){
    CameraServer.startAutomaticCapture();
  }

}
