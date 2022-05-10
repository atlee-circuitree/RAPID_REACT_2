// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;

import java.lang.Math;

/*
  NOTE: This class is currently NON-FUNCTIONAL
*/

public class DriveWithXboxOptimized extends CommandBase {

  private final Drivetrain drivetrain;
  private SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(0.8);
  private SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(0.8);
  private SlewRateLimiter slewRateLimiterZ = new SlewRateLimiter(0.2);
  
  private XboxController xbox;

  public boolean isTesting;

  private double forward;
  private double strafe;
  private double rotation;

  private int invertSpeed = 1;

  public static String driveWithXboxDashboard;
 
  public DriveWithXboxOptimized(Drivetrain dt, XboxController xboxController, boolean testing) {
    
    drivetrain = dt;
    xbox = xboxController;

    isTesting = testing;

    addRequirements(drivetrain);

  }

  private double invertAngle(double targetAngle){
    
    if(targetAngle > 0){
      targetAngle = targetAngle - 180;
    }
    if(targetAngle <= 0){
      targetAngle = targetAngle + 180;
    }
    
    return targetAngle;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    //Define robot target vector variables (X,Y,Z respectively)  
    //double forward = -slewRateLimiterX.calculate(RobotContainer.xbox.getLeftY());
    //double strafe = -slewRateLimiterY.calculate(RobotContainer.xbox.getLeftX());
    //double rotation = -slewRateLimiterZ.calculate(RobotContainer.xbox.getRightX());

    forward = xbox.getLeftY() * 0.6;
    strafe = xbox.getLeftX() * 0.6;
    rotation = xbox.getRightX() * 0.6;

    if(rotation != 0 && (forward != 0 || strafe != 0)){
      rotation = slewRateLimiterZ.calculate(xbox.getRightX()) * 0.6;
    }

    //Controller Deadband
    if(Math.abs(forward) < 0.05){
      forward = 0;
    }
    if(Math.abs(strafe) < 0.05){
      strafe = 0;
    }
    if(Math.abs(rotation) < 0.05){
      rotation = 0;
    }

    //Modify target values for field orientation (temp used to save calculations before original forward and strafe values are modified)
    double temp = forward * Math.cos(-drivetrain.getNavXOutputRadians()) + strafe * Math.sin(-drivetrain.getNavXOutputRadians()); 
    strafe = -forward * Math.sin(-drivetrain.getNavXOutputRadians()) + strafe * Math.cos(-drivetrain.getNavXOutputRadians()); 
    forward = temp;

    //Do some math to calculate the angles/sppeds needed to meet the target vectors
    //I don't have enough space to say what A,B,C and D actually represent, but the swerve documentation does it well 
    double A = strafe - (rotation * (Constants.wheelbase/Constants.drivetrainRadius));
    double B = strafe + (rotation * (Constants.wheelbase/Constants.drivetrainRadius));
    double C = forward - (rotation * (Constants.trackwidth/Constants.drivetrainRadius));
    double D = forward + (rotation * (Constants.trackwidth/Constants.drivetrainRadius));

    //Calculates module speeds
    double frontLeftSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    double frontRightSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    double rearLeftSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
    double rearRightSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));

    double frontLeftAngle = Math.atan2(B, C)*(180/Math.PI);
    double frontRightAngle = Math.atan2(B, D)*(180/Math.PI);
    double rearLeftAngle = Math.atan2(A, C)*(180/Math.PI);
    double rearRightAngle = Math.atan2(A, D)*(180/Math.PI);

    //Normalizes speeds (makes sure that none are > 1)
    double max = frontLeftSpeed;
    if(max < frontRightSpeed){
      max = frontRightSpeed;
    }
    if(max < rearLeftSpeed){
      max = rearLeftSpeed;
    } 
    if(max < rearRightSpeed){
      max = rearRightSpeed;
    }
    if(max > 1){
      frontLeftSpeed = frontLeftSpeed / max;
      frontRightSpeed = frontRightSpeed / max;
      rearLeftSpeed = rearLeftSpeed / max;
      rearRightSpeed = rearRightSpeed / max;
    }

    //Optimization code
    if(Math.abs(invertAngle(drivetrain.getRotEncoderValue(SwerveModule.FRONT_LEFT)) - frontLeftAngle) < Math.abs(drivetrain.getRotEncoderValue(SwerveModule.FRONT_LEFT) - frontLeftAngle)){
      frontLeftSpeed = frontLeftSpeed * -1;
      frontLeftAngle = invertAngle(frontLeftAngle);
    }
    if(Math.abs(invertAngle(drivetrain.getRotEncoderValue(SwerveModule.FRONT_RIGHT)) - frontRightAngle) < Math.abs(drivetrain.getRotEncoderValue(SwerveModule.FRONT_RIGHT) - frontRightAngle)){
      frontRightSpeed = frontRightSpeed * -1;
      frontRightAngle = invertAngle(frontRightAngle);
    }
    if(Math.abs(invertAngle(drivetrain.getRotEncoderValue(SwerveModule.REAR_LEFT)) - rearLeftAngle) < Math.abs(drivetrain.getRotEncoderValue(SwerveModule.REAR_LEFT) - rearLeftAngle)){
      rearLeftSpeed = rearLeftSpeed * -1;
      rearLeftAngle = invertAngle(rearLeftAngle);
    }
    if(Math.abs(invertAngle(drivetrain.getRotEncoderValue(SwerveModule.REAR_RIGHT)) - rearRightAngle) < Math.abs(drivetrain.getRotEncoderValue(SwerveModule.REAR_RIGHT) - rearRightAngle)){
      rearRightSpeed = rearRightSpeed * -1;
      rearRightAngle = invertAngle(rearRightAngle);
    }

    //Make SURE the robot stops whenthe joysticks are 0
    if(forward == 0 && strafe == 0 && rotation == 0){
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, 0);

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, frontLeftAngle, 0);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, frontRightAngle, 0);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, rearLeftAngle, 0);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, rearRightAngle, 0);
    }
    else{
      //Set angles for modules (change speed mod later if needed)
      //Original angle values
      //FL: B, D
      //FR: B, C
      //RL: A, D
      //RR: A, C
      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, frontLeftAngle, 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, frontRightAngle, 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, rearLeftAngle, 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, rearRightAngle, 1);

      //Set speeds for modules
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, frontLeftSpeed);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, frontRightSpeed);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, rearLeftSpeed);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, rearRightSpeed);
    }

    //Show important values on dashboard
    //driveWithXboxDashboard = "FL Module/" + "Speed: " + String.valueOf(frontLeftSpeed) + " Angle: " + String.valueOf(Math.atan2(B, C)*(180/Math.PI)) + ";";
    //driveWithXboxDashboard = driveWithXboxDashboard + "FR Module/" + "Speed: " + String.valueOf(frontRightSpeed) + " Angle: " + String.valueOf(Math.atan2(B, D)*(180/Math.PI)) + ";";
    //driveWithXboxDashboard = driveWithXboxDashboard + "RL Module/" + "Speed: " + String.valueOf(rearLeftSpeed) + " Angle: " + String.valueOf(Math.atan2(A, C)*(180/Math.PI)) + ";";
    //driveWithXboxDashboard = driveWithXboxDashboard + "RR Module/" + "Speed: " + String.valueOf(rearRightSpeed) + " Angle: " + String.valueOf(Math.atan2(A, D)*(180/Math.PI)) + ";";
    driveWithXboxDashboard = driveWithXboxDashboard + "NavX Yaw/" + String.valueOf(drivetrain.getNavXOutput()) + ";";


    //DEBUG
    if(xbox.getBackButtonPressed()){
      drivetrain.zeroNavXYaw();
      drivetrain.resetOdometry(new Pose2d(new Translation2d(0, new Rotation2d(0)), new Rotation2d(0)));
    }
  
  }  

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
