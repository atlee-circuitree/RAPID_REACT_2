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

public class AutoDriveByMeters extends CommandBase {

  private final Drivetrain drivetrain;
  private XboxController xbox;

  public boolean isTesting;

  private double forward;
  private double strafe;
  private double rotation;

  private double targetDistance;

  private boolean isFinished = false;

  public static String driveWithXboxDashboard;
 
  public AutoDriveByMeters(Drivetrain dt, double forwardSpeed, double strafeSpeed, double rotationSpeed, double targetDistanceMeters) {
    
    drivetrain = dt;

    forward = forwardSpeed;
    strafe = strafeSpeed;
    rotation = rotationSpeed;

    targetDistance = targetDistanceMeters;

    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

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
    double frontLeftSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    double frontRightSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    double rearLeftSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
    double rearRightSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

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

    //Make SURE the robot stops when the joysticks are 0
    if((RobotContainer.xbox.getLeftX() == 0 && RobotContainer.xbox.getLeftY() == 0 && RobotContainer.xbox.getRightX() == 0 && isTesting == false)){
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, 0);

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, Math.atan2(B, C)*(180/Math.PI), 0);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, Math.atan2(B, D)*(180/Math.PI), 0);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, Math.atan2(A, C)*(180/Math.PI), 0);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, Math.atan2(A, D)*(180/Math.PI), 0);
    }
    else{
      //Set angles for modules (change speed mod later if needed)
      //Original angle values
      //FL: B, D
      //FR: B, C
      //RL: A, D
      //RR: A, C
      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, Math.atan2(B, C)*(180/Math.PI), 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, Math.atan2(B, D)*(180/Math.PI), 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, Math.atan2(A, C)*(180/Math.PI), 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, Math.atan2(A, D)*(180/Math.PI), 1);

      //Set speeds for modules
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, -frontLeftSpeed);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, -frontRightSpeed);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, -rearLeftSpeed);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, -rearRightSpeed);
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
    
    
    
    return isFinished;
  }
}
