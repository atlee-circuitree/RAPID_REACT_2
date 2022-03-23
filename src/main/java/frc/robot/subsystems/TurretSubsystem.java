// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
   
  TalonSRX topShootMotor = null;
  TalonSRX bottomShootMotor = null;
  
  CANSparkMax turretMotor = null;
  Encoder turretEncoder = null;
  PWM pwmEncoder;
  AnalogEncoder turretCoder;

  public static String turretDashboard;

  public TurretSubsystem() {
 
    topShootMotor = new TalonSRX(Constants.topShooterMotorPort);
    bottomShootMotor = new TalonSRX(Constants.bottomShooterMotorPort);
    
    turretMotor = new CANSparkMax(Constants.turretMotorPort, MotorType.kBrushed);
    turretEncoder = new Encoder(0, 1);
    pwmEncoder = new PWM(0);
    turretCoder = new AnalogEncoder(0);
    
    

  }
  @Override
  public void periodic() {
  
    turretDashboard = "Encoder Position/" + turretEncoder.get() + ";";
    turretDashboard = "PWM Encoder Position/" + pwmEncoder.getRaw() + ";";
   
  }

  protected void useOutput(double output, double setpoint) {
    turretMotor.set(output);
  }

  public double getMeasurement() {
    return 0;
  }

  //public double getLastEncoder() {
  //  return encoder.lastValue;
  //}


  public void runTurretWithVelocity(double velocity) {
    //Close SZ shoot = Bottom * 1.3
    topShootMotor.set(ControlMode.Velocity, -velocity);
    bottomShootMotor.set(ControlMode.Velocity, velocity * 1.3);

  }

  public double getVelocity() {

    double velocity;

    velocity = topShootMotor.getSelectedSensorVelocity();

    return velocity;

  }

  public void turnTurret(double speed) {

    turretMotor.set(speed);

    //System.out.print("Encoder Pos");
    //System.out.println(turretEncoder.get());

  }
 
}
