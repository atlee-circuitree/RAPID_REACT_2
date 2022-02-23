// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class TurretSubsystem extends PIDSubsystem {

  // Giant Chunk of Copy-Paste Code
  // https://cdn.shopify.com/s/files/1/1518/8108/files/Armabot_A0085_Turret240_Encoder_Board_RevB.PDF
  /*
  public class AS5600EncoderPwm {
    private final SensorCollection sensors;
    private volatile int lastValue = Integer.MIN_VALUE;
    public AS5600EncoderPwm(SensorCollection sensors) {
    this.sensors = sensors;
    }
    public int getPwmPosition() {
    int raw = sensors.getPulseWidthRiseToFallUs();
    if (raw == 0) {
    int lastValue = this.lastValue;
    if (lastValue == Integer.MIN_VALUE) {
    return 0;
    }
    return lastValue;
    }
    int actualValue = Math.min(4096, raw - 128);
    lastValue = actualValue;
    return actualValue;
    }
   }
   */

  //private final WPI_TalonSRX yourTalon = new WPI_TalonSRX(1);
  //private final AS5600EncoderPwm encoder = new
  //AS5600EncoderPwm(yourTalon.getSensorCollection());
   
  TalonSRX topShootMotor = null;
  TalonSRX bottomShootMotor = null;
  
  CANSparkMax turretMotor = null;
  Encoder turretEncoder = null;
  
  
  public static String turretDashboard;

  public TurretSubsystem() {
 
    super(new PIDController(0, 0, 0));
    topShootMotor = new TalonSRX(Constants.topShooterMotorPort);
    bottomShootMotor = new TalonSRX(Constants.bottomShooterMotorPort);
    
    turretMotor = new CANSparkMax(Constants.turretMotorPort, MotorType.kBrushed);
    turretEncoder = new Encoder(0, 1);

    

  }
  @Override
  public void periodic() {
  
    turretDashboard = "Encoder PWM Position/" + turretEncoder.get() + ";";
  
  
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

    topShootMotor.set(ControlMode.Velocity, -velocity);
    bottomShootMotor.set(ControlMode.Velocity, velocity);

  }

  public void turnTurret(double speed) {

    turretMotor.set(speed);

    System.out.print("Encoder Pos");
    System.out.println(turretEncoder.get());

  }
 
}
