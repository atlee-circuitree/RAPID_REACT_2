// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LaunchVelocity;
 
public class TurretSubsystem extends SubsystemBase {
   
  TalonSRX topShootMotor = null;
  TalonSRX bottomShootMotor = null;
  CANSparkMax turretMotor = null;
  RelativeEncoder turretEncoder = null;
  double shuffleShooterTop = -SmartDashboard.getNumber("Custom Top Velocity", 0);
  double shuffleShooterBottom = SmartDashboard.getNumber("Custom Bottom Velocity", 0);

  public static String turretDashboard;

  public TurretSubsystem() {
 
    topShootMotor = new TalonSRX(Constants.topShooterMotorPort);
    bottomShootMotor = new TalonSRX(Constants.bottomShooterMotorPort);

    topShootMotor.setNeutralMode(NeutralMode.Coast);
    bottomShootMotor.setNeutralMode(NeutralMode.Coast);

    
    turretMotor = new CANSparkMax(Constants.turretMotorPort, MotorType.kBrushless);

    turretEncoder = turretMotor.getEncoder(Type.kHallSensor, 42);

    SmartDashboard.putNumber("Custom Top Velocity", 0);
    SmartDashboard.putNumber("Custom Bottom Velocity", 0);
  
  }

  @Override
  public void periodic() {

    
    SmartDashboard.putNumber("Turret Angle", getTurretEncoder());
    
    double smartVelocity = SmartDashboard.getNumber("Turret Velocity", 0);
    double smartBottomMotorMod = SmartDashboard.getNumber("Turret Bottom Velocity", 1);
  

  }

  public LaunchVelocity[] getDistanceToVelocityArray(){
    // 10 = 1.0 Meter
    // First launch velocity value is the top motor
    // robotDistance[meters in tenths] = new LaunchVelocity(top motor velocity, bottom motor velocity);

    LaunchVelocity robotDistance[];
    robotDistance = new LaunchVelocity[61]; 
    robotDistance[0] = new LaunchVelocity(0, 0);
    robotDistance[1] = new LaunchVelocity(2500, 6000);
    robotDistance[2] = new LaunchVelocity(2500, 6000);
    robotDistance[3] = new LaunchVelocity(2500, 6000);
    robotDistance[4] = new LaunchVelocity(2500, 6000);
    robotDistance[5] = new LaunchVelocity(2500, 6000);
    robotDistance[6] = new LaunchVelocity(2500, 6000);
    robotDistance[7] = new LaunchVelocity(2500, 6000);
    robotDistance[8] = new LaunchVelocity(2500, 6000);
    robotDistance[9] = new LaunchVelocity(2500, 6000);
    robotDistance[10] = new LaunchVelocity(2500, 6000);
    robotDistance[11] = new LaunchVelocity(2500, 6000);
    robotDistance[12] = new LaunchVelocity(2500, 6000);
    robotDistance[13] = new LaunchVelocity(2500, 6000);
    robotDistance[14] = new LaunchVelocity(3000, 12000);
    robotDistance[15] = new LaunchVelocity(4000, 10000);
    robotDistance[16] = new LaunchVelocity(4200, 10000);
    robotDistance[17] = new LaunchVelocity(4450, 10000);
    robotDistance[18] = new LaunchVelocity(4700, 10000);
    robotDistance[19] = new LaunchVelocity(4950, 10000);
    robotDistance[20] = new LaunchVelocity(5200, 10000);
    robotDistance[21] = new LaunchVelocity(5250, 10000);
    robotDistance[22] = new LaunchVelocity(5300, 10000);
    robotDistance[23] = new LaunchVelocity(5300, 10000);
    robotDistance[24] = new LaunchVelocity(5300, 10000);
    robotDistance[25] = new LaunchVelocity(5500, 10000);
    robotDistance[26] = new LaunchVelocity(5800, 10000);
    robotDistance[27] = new LaunchVelocity(6100, 10000);
    robotDistance[28] = new LaunchVelocity(6700, 9300);
    robotDistance[29] = new LaunchVelocity(7000, 9300);
    robotDistance[30] = new LaunchVelocity(7700, 8500);
    robotDistance[31] = new LaunchVelocity(8600, 8000);
    robotDistance[32] = new LaunchVelocity(8800, 8000);
    robotDistance[33] = new LaunchVelocity(8900, 8000);
    robotDistance[34] = new LaunchVelocity(9000, 8000);
    robotDistance[35] = new LaunchVelocity(9300, 8000);
    robotDistance[36] = new LaunchVelocity(9400, 8000);
    robotDistance[37] = new LaunchVelocity(9600, 8000);
    robotDistance[38] = new LaunchVelocity(9800, 8000);
    robotDistance[39] = new LaunchVelocity(10300, 8000);
    robotDistance[40] = new LaunchVelocity(11200, 7800);
    robotDistance[41] = new LaunchVelocity(11500, 7600);
    robotDistance[42] = new LaunchVelocity(11600, 7600);
    robotDistance[43] = new LaunchVelocity(12900, 6700);
    robotDistance[44] = new LaunchVelocity(13200, 6700);
    robotDistance[45] = new LaunchVelocity(0, 0);
    robotDistance[46] = new LaunchVelocity(0, 0);
    robotDistance[47] = new LaunchVelocity(0, 0);
    robotDistance[48] = new LaunchVelocity(0, 0);
    robotDistance[49] = new LaunchVelocity(0, 0);
    robotDistance[50] = new LaunchVelocity(0, 0);
    robotDistance[51] = new LaunchVelocity(0, 0);
    robotDistance[52] = new LaunchVelocity(0, 0);
    robotDistance[53] = new LaunchVelocity(0, 0);
    robotDistance[54] = new LaunchVelocity(0, 0);
    robotDistance[55] = new LaunchVelocity(0, 0);
    robotDistance[56] = new LaunchVelocity(0, 0);
    robotDistance[57] = new LaunchVelocity(0, 0);
    robotDistance[58] = new LaunchVelocity(0, 0);
    robotDistance[59] = new LaunchVelocity(0, 0);
    robotDistance[60] = new LaunchVelocity(0, 0);
        return robotDistance;
  }

  protected void useOutput(double output, double setpoint) {

    turretMotor.set(output);

  }

  
  public double getTurretEncoder() {

    return turretEncoder.getPosition();

  }

  public double getMeasurement() {

    return 0;

  }

  public void runTurretWithVelocity(double topVelocity, double bottomVelocity) {

    topShootMotor.set(ControlMode.Velocity, -topVelocity);
    bottomShootMotor.set(ControlMode.Velocity, bottomVelocity);

  }

  public boolean checkShootVelocity(double topVelocity, double bottomVelocity, Timer timer, double timeout) {

    if (topShootMotor.getSelectedSensorVelocity() > topVelocity - 100 || timer.get() > timeout) {
      
      return true;

    } else {

      return false;

    }

  }


  public void killTurretMotors(){

    topShootMotor.set(ControlMode.PercentOutput, 0);
    bottomShootMotor.set(ControlMode.PercentOutput, 0);

  }

  public void runTurretWithMPSandShuffle() {

    //topShootMotor.set(ControlMode.Velocity, -metersPerSecondtoVelocity(SmartDashboard.getNumber("Turret Velocity", 0)));
    //bottomShootMotor.set(ControlMode.Velocity, metersPerSecondtoVelocity(SmartDashboard.getNumber("Turret Bottom Velocity", 0)));

  }

  public double getShuffleTopMotor() {

    return SmartDashboard.getNumber("Custom Top Velocity", 0);
    
  }

  public double getShuffleBottomMotor() {

    return SmartDashboard.getNumber("Custom Bottom Velocity", 0);
    
  }

  public double returnTopMotorWithVelocity() {
    //Cose SZ shoot = Bottom * 1.3
    return -topShootMotor.getSelectedSensorVelocity();
    
  }

  public double returnBottomMotorWithVelocity() {
    //Close SZ shoot = Bottom * 1.3
    return bottomShootMotor.getSelectedSensorVelocity();
    
  }

  public void turnTurret(double speed) {

    turretMotor.set(speed);
  
  }
  
  public double metersPerSecondtoVelocity(double metersPerSecond){
    
    double degreesPerSecond = (metersPerSecond/0.1016)*(180/Math.PI);
    double positionChangePer100ms = (degreesPerSecond * 44.9)/10;

    return positionChangePer100ms;
  }

  public double[] distanceToShooterMatrix(double distanceIn){

    //ENTER VALUES IN LOW TO HIGH DISTANCE
    double[] distanceList = {2, 2.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double[] topVelocityList = {2.3, 2.3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double[] bottomVelocityList = {3.8, 3.8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    for(int i = 0; i < distanceList.length; i++){
      
      try{
        //If the distance in is bigger than the current value being checked, but smaller than the next,
        //it means that the distance in is between one of those values, meaning that we will return one of those 2 points
        if(distanceIn > distanceList[i] && distanceIn < distanceList[i+1]){

          //If the distance between the distance in and the 1st value in consideration is smaller than the distance between
          //the distance in and the 2nd value in consideration, return the velocities corresponding to the 1st value
          if(Math.abs(distanceIn - distanceList[i]) < Math.abs(distanceIn - distanceList[i+1])){
            double[] returnValues = {topVelocityList[i], bottomVelocityList[i]};
            return returnValues;
          }
          //Otherwise return the velocities corresponding to the 2nd value
          else{
            double[] returnValues = {topVelocityList[i+1], bottomVelocityList[i+1]};
            return returnValues;
          }

        }
      }
      //If we somehow get a distance larger than any of the ones we have in the table (causing an error),
      //output the largest value we have
      catch(ArrayIndexOutOfBoundsException exception){
        double[] returnValues = {topVelocityList[topVelocityList.length], bottomVelocityList[bottomVelocityList.length]};
        return returnValues;
      }

    }
    
    return null;

  }
  
 
}  