// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.kauailabs.navx.frc.AHRS;


public class Drivetrain extends SubsystemBase {

  TalonFX frontLeftDrvMotor;
  TalonFX frontRightDrvMotor;
  TalonFX rearLeftDrvMotor;
  TalonFX rearRightDrvMotor;

  TalonFX frontLeftRotMotor;
  TalonFX frontRightRotMotor;
  TalonFX rearLeftRotMotor;
  TalonFX rearRightRotMotor;

  CANCoder frontLeftRotEncoder;
  CANCoder frontRightRotEncoder;
  CANCoder rearLeftRotEncoder;
  CANCoder rearRightRotEncoder;

  PIDController frontLeftPID;
  PIDController frontRightPID;
  PIDController rearLeftPID;
  PIDController rearRightPID;

  AHRS navx;

  SwerveDriveOdometry odometry;

  public static String drivetrainDashboard;

  public Drivetrain() {
    frontLeftDrvMotor = new TalonFX(Constants.frontLeftDrvMotorPort);
    frontRightDrvMotor = new TalonFX(Constants.frontRightDrvMotorPort);
    rearLeftDrvMotor = new TalonFX(Constants.rearLeftDrvMotorPort);
    rearRightDrvMotor = new TalonFX(Constants.rearRightDrvMotorPort);

    //This *should* be selecting the inbuilt talon encoder as the output device
    //Dunno what 1 and 50 are suppposed to be
    frontLeftDrvMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 50);
    frontRightDrvMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 50);
    rearLeftDrvMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 50);
    rearRightDrvMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 50);

    frontLeftRotMotor = new TalonFX(Constants.frontLeftRotMotorPort);
    frontRightRotMotor = new TalonFX(Constants.frontRightRotMotorPort);
    rearLeftRotMotor = new TalonFX(Constants.rearLeftRotMotorPort);
    rearRightRotMotor = new TalonFX(Constants.rearRightRotMotorPort);

    frontLeftDrvMotor.setNeutralMode(NeutralMode.Brake);
    frontRightDrvMotor.setNeutralMode(NeutralMode.Brake);
    rearLeftDrvMotor.setNeutralMode(NeutralMode.Brake);
    rearRightDrvMotor.setNeutralMode(NeutralMode.Brake);

    frontLeftRotMotor.setNeutralMode(NeutralMode.Brake);
    frontRightRotMotor.setNeutralMode(NeutralMode.Brake);
    rearLeftRotMotor.setNeutralMode(NeutralMode.Brake);
    rearRightRotMotor.setNeutralMode(NeutralMode.Brake);

    frontLeftRotEncoder = new CANCoder(Constants.frontLeftRotEncoderPort);
    frontRightRotEncoder = new CANCoder(Constants.frontRightRotEncoderPort);
    rearLeftRotEncoder = new CANCoder(Constants.rearLeftRotEncoderPort);
    rearRightRotEncoder = new CANCoder(Constants.rearRightRotEncoderPort);

    frontLeftRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    frontRightRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rearLeftRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rearRightRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    frontLeftRotEncoder.configMagnetOffset(0);
    frontRightRotEncoder.configMagnetOffset(0);
    rearLeftRotEncoder.configMagnetOffset(0);
    rearRightRotEncoder.configMagnetOffset(0);

    frontLeftRotEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    frontRightRotEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rearLeftRotEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rearRightRotEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    

    //We are using getAboslutePosition(), but just in case
    frontLeftRotEncoder.setPosition(0);
    frontRightRotEncoder.setPosition(0);
    rearLeftRotEncoder.setPosition(0);
    rearRightRotEncoder.setPosition(0);

    //Best values so far: 0.65, 0.065, 0.01
    frontLeftPID = new PIDController(Constants.rotPID_P, Constants.rotPID_I, Constants.rotPID_D);
    frontRightPID = new PIDController(Constants.rotPID_P, Constants.rotPID_I, Constants.rotPID_D);
    rearLeftPID = new PIDController(Constants.rotPID_P, Constants.rotPID_I, Constants.rotPID_D);
    rearRightPID = new PIDController(Constants.rotPID_P, Constants.rotPID_I, Constants.rotPID_D);

    frontLeftPID.enableContinuousInput(-180, 180);
    frontRightPID.enableContinuousInput(-180, 180);
    rearLeftPID.enableContinuousInput(-180, 180);
    rearRightPID.enableContinuousInput(-180, 180);

    frontLeftPID.setTolerance(1.0);
    frontRightPID.setTolerance(1.0);
    rearLeftPID.setTolerance(1.0);
    rearRightPID.setTolerance(1.0);
    
    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();

    odometry = new SwerveDriveOdometry(Constants.driveKinematics, navx.getRotation2d());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Updates odometry
    odometry.update(navx.getRotation2d(), 
    new SwerveModuleState(positionChangePer100msToMetersPerSecond(frontLeftDrvMotor.getSelectedSensorVelocity()), 
    Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.FRONT_LEFT))),
    
    new SwerveModuleState(positionChangePer100msToMetersPerSecond(frontRightDrvMotor.getSelectedSensorVelocity()), 
    Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.FRONT_RIGHT))),
    
    new SwerveModuleState(positionChangePer100msToMetersPerSecond(rearLeftDrvMotor.getSelectedSensorVelocity()), 
    Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.REAR_LEFT))),
    
    new SwerveModuleState(positionChangePer100msToMetersPerSecond(rearRightDrvMotor.getSelectedSensorVelocity()), 
    Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.REAR_RIGHT))));

    drivetrainDashboard = "frontLeft rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontLeft PID Output/" + getRotPIDOutput(SwerveModule.FRONT_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontLeft PID Setpoint/" + frontLeftPID.getSetpoint() + ";";

    drivetrainDashboard = drivetrainDashboard + "frontRight rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontRight PID/" + getRotPIDOutput(SwerveModule.FRONT_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontRight PID Setpoint/" + frontRightPID.getSetpoint() + ";";

    drivetrainDashboard = drivetrainDashboard + "rearLeft rot encoder/" + getRotEncoderValue(SwerveModule.REAR_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearLeft PID/" + getRotPIDOutput(SwerveModule.REAR_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearleft PID Setpoint/" + rearLeftPID.getSetpoint() + ";";

    drivetrainDashboard = drivetrainDashboard + "rearRight rot encoder/" + getRotEncoderValue(SwerveModule.REAR_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearRight PID/" + getRotPIDOutput(SwerveModule.REAR_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearRight PID Setpoint/" + rearRightPID.getSetpoint() + ";"; 

    drivetrainDashboard = drivetrainDashboard + "odometry X/" + odometry.getPoseMeters().getX() + ";";
    drivetrainDashboard = drivetrainDashboard + "odometry Y/" + odometry.getPoseMeters().getY() + ";";
    drivetrainDashboard = drivetrainDashboard + "odometry Z/" + odometry.getPoseMeters().getRotation().getDegrees() + ";";

    drivetrainDashboard = drivetrainDashboard + "FL Encoder Position/" + getDriveEncoder(SwerveModule.FRONT_LEFT) + ";";

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //------------------------------------------------------------------------------------------------------------------------------------
  //ENUMS
  //------------------------------------------------------------------------------------------------------------------------------------
  public enum SwerveModule{
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
  }

  public enum Motors{
    FRONT_LEFT_ROT, FRONT_RIGHT_ROT, REAR_LEFT_ROT, REAR_RIGHT_ROT, FRONT_LEFT_DRV, FRONT_RIGHT_DRV, REAR_LEFT_DRV, REAR_RIGHT_DRV
  }

  //------------------------------------------------------------------------------------------------------------------------------------
  //DRIVE/ROTATION
  //------------------------------------------------------------------------------------------------------------------------------------


  //Powers all drive motors on the module
  public void driveAllModules(double speed){
    
    frontLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    frontRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    rearLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    rearRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
  
  }

  //Stops all drive and rotational motors
  public void killAllModules(){
    
    frontLeftDrvMotor.set(TalonFXControlMode.PercentOutput, 0);
    frontRightDrvMotor.set(TalonFXControlMode.PercentOutput, 0);
    rearLeftDrvMotor.set(TalonFXControlMode.PercentOutput, 0);
    rearRightDrvMotor.set(TalonFXControlMode.PercentOutput, 0);

    frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    rearRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
  
  }


  //Universal function for providing power to any one drivetrain motor
  public void rotateMotor(Motors motor, double speed){
    if(motor == Motors.FRONT_LEFT_ROT){
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.FRONT_RIGHT_ROT){
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.REAR_LEFT_ROT){
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.REAR_RIGHT_ROT){
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.FRONT_LEFT_DRV){
      frontLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.FRONT_RIGHT_DRV){
      frontRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.REAR_LEFT_DRV){
      rearLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.REAR_RIGHT_DRV){
      rearRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
  }

  //Provides power to the rotational module motors using a PID
  public void rotateModule(SwerveModule module, double targetDegrees, double speedMod){
    
    setRotPIDSetpoint(module, targetDegrees);

    if(module == SwerveModule.FRONT_LEFT){
      if(frontLeftPID.atSetpoint()){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, getRotPIDOutput(module)/180 * speedMod);
      }
    }
    if(module == SwerveModule.FRONT_RIGHT){
      if(frontRightPID.atSetpoint()){
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, getRotPIDOutput(module)/180 * speedMod);
      }
    }
    if(module == SwerveModule.REAR_LEFT){
      if(rearLeftPID.atSetpoint()){
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, getRotPIDOutput(module)/180 * speedMod);
      }
    }
    if(module == SwerveModule.REAR_RIGHT){
      if(rearRightPID.atSetpoint()){
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, getRotPIDOutput(module)/180 * speedMod);
      }
    }

  }



  

  //------------------------------------------------------------------------------------------------------------------------------------
  //SENSORS
  //------------------------------------------------------------------------------------------------------------------------------------

  //Returns the rotational encoder value (-180,180) of a module, modified by an offset value
  public double getRotEncoderValue(SwerveModule module){

    double encoderValue = 0;

    //Assigns offset encoder absolute position based on input SwerveModule parameter
    if(module == SwerveModule.FRONT_LEFT){
      encoderValue = frontLeftRotEncoder.getAbsolutePosition() - Constants.frontLeftEncoderOffset;
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      encoderValue = frontRightRotEncoder.getAbsolutePosition() - Constants.frontRightEncoderOffset;
    }
    else if(module == SwerveModule.REAR_LEFT){
      encoderValue = rearLeftRotEncoder.getAbsolutePosition() - Constants.rearLeftEncoderOffset;
    }
    else if(module == SwerveModule.REAR_RIGHT){
      encoderValue = rearRightRotEncoder.getAbsolutePosition() - Constants.rearRightEncoderOffset;
    }
    else{
      return 0;
    }


    //Deals with offset loop bug
    if(encoderValue < 0){
      encoderValue = encoderValue + 360;
    }
    
    
    //Signs values ([-180,180] not [0,360])
    if(encoderValue > 180){
      encoderValue = encoderValue - 360;
    }
    

    return encoderValue;
  }

  //Returns the rotational encoder value without any modification
  public double getAbsoluteRotEncoderValue(SwerveModule module){
  
    //Assigns offset encoder absolute position based on input SwerveModule parameter
    if(module == SwerveModule.FRONT_LEFT){
      return frontLeftRotEncoder.getAbsolutePosition();
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      return frontRightRotEncoder.getAbsolutePosition();
    }
    else if(module == SwerveModule.REAR_LEFT){
      return rearLeftRotEncoder.getAbsolutePosition();
    }
    else if(module == SwerveModule.REAR_RIGHT){
      return rearRightRotEncoder.getAbsolutePosition();
    }
    else{
      return 0;
    }
  }

  //Returns the inbuilt encoders on the drive falcons
  public double getDriveEncoder(SwerveModule module){
    
    if(module == SwerveModule.FRONT_LEFT){
      return frontLeftDrvMotor.getSelectedSensorPosition();
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      return frontRightDrvMotor.getSelectedSensorPosition();
    }
    else if(module == SwerveModule.REAR_LEFT){
      return rearLeftDrvMotor.getSelectedSensorPosition();
    }
    else if(module == SwerveModule.REAR_RIGHT){
      return rearRightDrvMotor.getSelectedSensorPosition();
    }
    else{
      return 0;
    }
  }

  //Resets the drive falcon encoders
  public void resetDriveEncoders(){
    frontLeftDrvMotor.setSelectedSensorPosition(0);
    frontRightDrvMotor.setSelectedSensorPosition(0);
    rearLeftDrvMotor.setSelectedSensorPosition(0);
    rearRightDrvMotor.setSelectedSensorPosition(0);
  }
 
  //Returns the PID value based on the rotational encoder value
  //The PID output is clamped between -180 and 180, and cannot return a value that is too small to power the motor (using a deadband)
  public double getRotPIDOutput(SwerveModule module){
    if(module == SwerveModule.FRONT_LEFT){
      double measurement = MathUtil.clamp(frontLeftPID.calculate(getRotEncoderValue(SwerveModule.FRONT_LEFT)), -180, 180);
      
      //Deadband
      if(measurement/180 < Constants.rotPIDMinValue && measurement/180 > 0){
        measurement = Constants.rotPIDMinValue*180;
      }
      else if(measurement/180 > -Constants.rotPIDMinValue && measurement/180 < 0){
        measurement = -Constants.rotPIDMinValue*180;
      }
      
      return measurement;
     
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      double measurement = frontRightPID.calculate(getRotEncoderValue(SwerveModule.FRONT_RIGHT));
      
      if(measurement/180 < Constants.rotPIDMinValue && measurement/180 > 0){
        measurement = Constants.rotPIDMinValue*180;
      }
      else if(measurement/180 > -Constants.rotPIDMinValue && measurement/180 < 0){
        measurement = -Constants.rotPIDMinValue*180;
      }
      
      return MathUtil.clamp(measurement, -180, 180);
      
    }
    else if(module == SwerveModule.REAR_LEFT){
      double measurement = rearLeftPID.calculate(getRotEncoderValue(SwerveModule.REAR_LEFT));
      
      if(measurement/180 < Constants.rotPIDMinValue && measurement/180 > 0){
        measurement = Constants.rotPIDMinValue*180;
      }
      else if(measurement/180 > -Constants.rotPIDMinValue && measurement/180 < 0){
        measurement = -Constants.rotPIDMinValue*180;
      }
      
      return MathUtil.clamp(measurement, -180, 180);
    
    }
    else if(module == SwerveModule.REAR_RIGHT){
      double measurement = rearRightPID.calculate(getRotEncoderValue(SwerveModule.REAR_RIGHT));
      
      if(measurement/180 < Constants.rotPIDMinValue && measurement/180 > 0){
        measurement = Constants.rotPIDMinValue*180;
      }
      else if(measurement/180 > -Constants.rotPIDMinValue && measurement/180 < 0){
        measurement = -Constants.rotPIDMinValue*180;
      }

      return MathUtil.clamp(measurement, -180, 180);
    }
    else{
      return 0;
    }
  }

  //Sets the PID setpoint
  public void setRotPIDSetpoint(SwerveModule module, double setpoint){
    if(module == SwerveModule.FRONT_LEFT){
      frontLeftPID.setSetpoint(setpoint);
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      frontRightPID.setSetpoint(setpoint);
    }
    else if(module == SwerveModule.REAR_LEFT){
      rearLeftPID.setSetpoint(setpoint);
    }
    else if(module == SwerveModule.REAR_RIGHT){
      rearRightPID.setSetpoint(setpoint);
    }
    else{
    }
  }
  

  public double getNavXOutput(){
    return navx.getYaw();
  }
  public double getNavXOutputRadians(){
    return Math.toRadians(navx.getYaw());
  }
  public void zeroNavXYaw(){
    navx.zeroYaw();
  }


  //------------------------------------------------------------------------------------------------------------------------------------
  //OTHER FUNCTIONS
  //------------------------------------------------------------------------------------------------------------------------------------

  //Converts the dumb falcon velocity unit to meters per second
  public double positionChangePer100msToMetersPerSecond(double posChangePer100ms){
    
    //posChangePer100ms/10 = posChangePerSecond

    //posChangePerSecond/46.4213 = degreesPerSecond (46.4213 being the encoder counts per degree)
    //(degreesPerSecond * PI/180) = radiansPerSecond
    //radiansPerSecond*0.0508 = metersPerSecond (0.0508 being the radius of the wheel in meters)

    double degreesPerSecond = (posChangePer100ms*10)/46.4213;
    double metersPerSecond = (degreesPerSecond*(Math.PI/180))*0.0508;

    return metersPerSecond;
  }

  //Clamps values without having to do absolute value
  public double mapValues(double value, double highest, double lowest){
    if(value >= 0){
      return MathUtil.clamp(value, lowest, highest);
    }
    else{
      return MathUtil.clamp(value, -highest, -lowest);
  }
}




//AUTO STUFF

public Pose2d getPose(){
  return odometry.getPoseMeters();
}

public double getOdometryX(){
  return odometry.getPoseMeters().getX();
}
public double getOdometryY(){
  return odometry.getPoseMeters().getY();
}
public double getOdometryZ(){
  return odometry.getPoseMeters().getRotation().getDegrees();
}

public void resetOdometry(Pose2d pose2d) {
  odometry.resetPosition(pose2d, navx.getRotation2d());
}

public void updateOdometry(){
    odometry.update(navx.getRotation2d(), 
    new SwerveModuleState(positionChangePer100msToMetersPerSecond(frontLeftDrvMotor.getSelectedSensorVelocity()), 
    Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.FRONT_LEFT))),
    
    new SwerveModuleState(positionChangePer100msToMetersPerSecond(frontRightDrvMotor.getSelectedSensorVelocity()), 
    Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.FRONT_RIGHT))),
    
    new SwerveModuleState(positionChangePer100msToMetersPerSecond(rearLeftDrvMotor.getSelectedSensorVelocity()), 
    Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.REAR_LEFT))),
    
    new SwerveModuleState(positionChangePer100msToMetersPerSecond(rearRightDrvMotor.getSelectedSensorVelocity()), 
    Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.REAR_RIGHT))));
}

public void setSwerveModuleStates(SwerveModuleState[] targetState){
  rotateModule(SwerveModule.FRONT_LEFT, targetState[0].angle.getDegrees(), 1);
  rotateModule(SwerveModule.FRONT_RIGHT, targetState[1].angle.getDegrees(), 1);
  rotateModule(SwerveModule.REAR_LEFT, targetState[2].angle.getDegrees(), 1);
  rotateModule(SwerveModule.REAR_RIGHT, targetState[3].angle.getDegrees(), 1);

  rotateMotor(Motors.FRONT_LEFT_DRV, targetState[0].speedMetersPerSecond/4.14528);
  rotateMotor(Motors.FRONT_RIGHT_DRV, targetState[1].speedMetersPerSecond/4.14528);
  rotateMotor(Motors.REAR_LEFT_DRV, targetState[2].speedMetersPerSecond/4.14528);
  rotateMotor(Motors.REAR_RIGHT_DRV, targetState[3].speedMetersPerSecond/4.14528);
}







}

