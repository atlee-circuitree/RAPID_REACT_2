// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
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

    //We are using getAboslutePosition(), but just in case
    frontLeftRotEncoder.setPosition(0);
    frontRightRotEncoder.setPosition(0);
    rearLeftRotEncoder.setPosition(0);
    rearRightRotEncoder.setPosition(0);

    //not sure if these are the perfect values, but they work good enough, so.....
    frontLeftPID = new PIDController(0.0075, 0.00, 0.00);
    frontRightPID = new PIDController(0.0075, 0.00, 0.00);
    rearLeftPID = new PIDController(0.0075, 0.00, 0.00);
    rearRightPID = new PIDController(0.0075, 0.00, 0.00);

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
    new SwerveModuleState(frontLeftDrvMotor.getSelectedSensorVelocity(), Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.FRONT_LEFT))),
    new SwerveModuleState(frontRightDrvMotor.getSelectedSensorVelocity(), Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.FRONT_RIGHT))),
    new SwerveModuleState(rearLeftDrvMotor.getSelectedSensorVelocity(), Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.REAR_LEFT))),
    new SwerveModuleState(rearRightDrvMotor.getSelectedSensorVelocity(), Rotation2d.fromDegrees(getRotEncoderValue(SwerveModule.REAR_RIGHT))));

    drivetrainDashboard = "frontLeft rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontLeft PID/" + getRotPIDOutput(SwerveModule.FRONT_LEFT) + ";";

    drivetrainDashboard = drivetrainDashboard + "frontRight rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontRight PID/" + getRotPIDOutput(SwerveModule.FRONT_RIGHT) + ";";

    drivetrainDashboard = drivetrainDashboard + "rearLeft rot encoder/" + getRotEncoderValue(SwerveModule.REAR_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearLeft PID/" + getRotPIDOutput(SwerveModule.REAR_LEFT) + ";";

    drivetrainDashboard = drivetrainDashboard + "rearRight rot encoder/" + getRotEncoderValue(SwerveModule.REAR_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearRight PID/" + getRotPIDOutput(SwerveModule.REAR_RIGHT) + ";"; 

    drivetrainDashboard = drivetrainDashboard + "odometry X/" + odometry.getPoseMeters().getX() + ";";
    drivetrainDashboard = drivetrainDashboard + "odometry Y/" + odometry.getPoseMeters().getY() + ";";
    drivetrainDashboard = drivetrainDashboard + "odometry Z/" + odometry.getPoseMeters().getRotation().getDegrees();

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

  public void rotateAllModulesNonLinear(double targetDegrees, double speed){

    frontLeftPID.setSetpoint(targetDegrees);
    frontRightPID.setSetpoint(targetDegrees);
    rearLeftPID.setSetpoint(targetDegrees);
    rearRightPID.setSetpoint(targetDegrees);

    //FRONT LEFT
    if(frontLeftPID.atSetpoint()){
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else{
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_LEFT), -speed, speed));
    }

    //FRONT RIGHT
    if(frontRightPID.atSetpoint()){
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else{
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_RIGHT), -speed, speed));
    }

    //REAR LEFT
    if(rearLeftPID.atSetpoint()){
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else{
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_LEFT), -speed, speed));
    }

    //REAR RIGHT
    if(rearRightPID.atSetpoint()){
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else{
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_RIGHT), -speed, speed));
    }

  } 


  //The only difference between rotateAllModulesNonLinear() and rotateAllModulesLinear() is that the non linear is meant to be
  //called in a bigger loop, and the linear one makes its own loop

  public void rotateAllModulesLinear(double targetDegrees, double speed){

    frontLeftPID.setSetpoint(targetDegrees);
    frontRightPID.setSetpoint(targetDegrees);
    rearLeftPID.setSetpoint(targetDegrees);
    rearRightPID.setSetpoint(targetDegrees);

    while(!frontLeftPID.atSetpoint() || !frontRightPID.atSetpoint() || !rearLeftPID.atSetpoint() || !rearRightPID.atSetpoint()){

      //FRONT LEFT
      if(frontLeftPID.atSetpoint()){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_LEFT), -speed, speed));
      }

      //FRONT RIGHT
      if(frontRightPID.atSetpoint()){
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_RIGHT), -speed, speed));
      }

      //REAR LEFT
      if(rearLeftPID.atSetpoint()){
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_LEFT), -speed, speed));
      }

      //REAR RIGHT
      if(rearRightPID.atSetpoint()){
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_RIGHT), -speed, speed));
      }
    }

  }


  public void driveAllModulesNonLinear(double speed){
    
    frontLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    frontRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    rearLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    rearRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
  
  }

  //Add other motors as needed, just make sure to put them in the enum too
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

  public void rotateModuleNonLinear(SwerveModule module, double targetDegrees, double speed){

    if(module == SwerveModule.FRONT_LEFT){
      frontLeftPID.setSetpoint(targetDegrees);
      if(frontLeftPID.atSetpoint()){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_LEFT), -speed, speed));
      }
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      frontRightPID.setSetpoint(targetDegrees);
      if(frontRightPID.atSetpoint()){
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_RIGHT), -speed, speed));
      }
    }
    else if(module == SwerveModule.REAR_LEFT){
      rearLeftPID.setSetpoint(targetDegrees);
      if(rearLeftPID.atSetpoint()){
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_LEFT), -speed, speed));
      }
    }
    else if(module == SwerveModule.REAR_RIGHT){
      rearRightPID.setSetpoint(targetDegrees);
      if(rearRightPID.atSetpoint()){
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_RIGHT), -speed, speed));
      }
    }

  }



  public void rotateModuleLinear(SwerveModule module, double targetDegrees, double speed){

    if(module == SwerveModule.FRONT_LEFT){
      frontLeftPID.setSetpoint(targetDegrees);
      while(!frontLeftPID.atSetpoint()){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_LEFT), -speed, speed));
      }
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      frontRightPID.setSetpoint(targetDegrees);
      while(!frontRightPID.atSetpoint()){
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_RIGHT), -speed, speed));
      }
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else if(module == SwerveModule.REAR_LEFT){
      rearLeftPID.setSetpoint(targetDegrees);
      while(!rearLeftPID.atSetpoint()){
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_LEFT), -speed, speed));
      }
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else if(module == SwerveModule.REAR_RIGHT){
      rearRightPID.setSetpoint(targetDegrees);
      while(!rearRightPID.atSetpoint()){
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_RIGHT), -speed, speed));
      }
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

  }


  //------------------------------------------------------------------------------------------------------------------------------------
  //SENSORS
  //------------------------------------------------------------------------------------------------------------------------------------

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
  public double getRotPIDOutput(SwerveModule module){
    if(module == SwerveModule.FRONT_LEFT){
      return frontLeftPID.calculate(getRotEncoderValue(SwerveModule.FRONT_LEFT));
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      return frontRightPID.calculate(getRotEncoderValue(SwerveModule.FRONT_RIGHT));
    }
    else if(module == SwerveModule.REAR_LEFT){
      return rearLeftPID.calculate(getRotEncoderValue(SwerveModule.REAR_LEFT));
    }
    else if(module == SwerveModule.REAR_RIGHT){
      return rearRightPID.calculate(getRotEncoderValue(SwerveModule.REAR_RIGHT));
    }
    else{
      return 0;
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

  public double mapValues(double value, double highest, double lowest){
    if(value >= 0){
      return MathUtil.clamp(value, lowest, highest);
    }
    else{
      return MathUtil.clamp(value, -highest, -lowest);
  }
}

public double customPID(double target){
  return Math.pow((target - getNavXOutput())/180, 2);
}


  public String sendDashboard(){
    
    String drivetrainDashboard;

    drivetrainDashboard = "frontLeft rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontLeft PID/" + getRotPIDOutput(SwerveModule.FRONT_LEFT) + ";";

    drivetrainDashboard = drivetrainDashboard + "frontRight rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontRight PID/" + getRotPIDOutput(SwerveModule.FRONT_RIGHT) + ";";

    drivetrainDashboard = drivetrainDashboard + "rearLeft rot encoder/" + getRotEncoderValue(SwerveModule.REAR_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearLeft PID/" + getRotPIDOutput(SwerveModule.REAR_LEFT) + ";";

    drivetrainDashboard = drivetrainDashboard + "rearRight rot encoder/" + getRotEncoderValue(SwerveModule.REAR_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearRight PID/" + getRotPIDOutput(SwerveModule.REAR_RIGHT);

    return drivetrainDashboard;
  }







//AUTO STUFF

public Pose2d getPose() {
  return odometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose) {
  odometry.resetPosition(pose, navx.getRotation2d());
}

public void setSwerveModuleStates(SwerveModuleState[] targetState){
  rotateModuleNonLinear(SwerveModule.FRONT_LEFT, targetState[0].angle.getDegrees(), targetState[0].speedMetersPerSecond);
  rotateModuleNonLinear(SwerveModule.FRONT_RIGHT, targetState[1].angle.getDegrees(), targetState[1].speedMetersPerSecond);
  rotateModuleNonLinear(SwerveModule.REAR_LEFT, targetState[2].angle.getDegrees(), targetState[2].speedMetersPerSecond);
  rotateModuleNonLinear(SwerveModule.REAR_RIGHT, targetState[3].angle.getDegrees(), targetState[3].speedMetersPerSecond);
}







}

