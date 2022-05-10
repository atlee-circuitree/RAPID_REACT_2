// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;

import java.lang.Math;

//What this file essentially does is drive the robot in autonomous using teleop
//It "sees" the field using odometry, then moves certain distances specified by parameters
//This is really buggy and patched together, as I made it basically the day before districts, but it works


public class AutoDriveByMeters extends CommandBase {

  private final Drivetrain drivetrain;
  

  private double forward = 0;
  private double strafe = 0;
  private double rotation = 0;


  private double targetDistanceX;
  private double targetDistanceY;
  private double targetAngleZ;

  private boolean isFinished = false;

  public AutoDriveByMeters(Drivetrain dt, double forwardSpeed, double strafeSpeed, double rotationSpeed, double distanceX, double distanceY, double angleZ) {
    
    drivetrain = dt;

    forward = forwardSpeed;
    strafe = strafeSpeed;
    rotation = rotationSpeed;

    targetDistanceX = distanceX;
    targetDistanceY = distanceY;
    targetAngleZ = angleZ;

    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {
    drivetrain.resetDriveEncoders();
    drivetrain.zeroNavXYaw();
    drivetrain.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }

  @Override
  public void execute() {

    drivetrain.updateOdometry();

    //If the drivetrain has finished the x/y/z component of the move, stop moving in that direction so that you don't overshoot
    if(Math.abs(drivetrain.getOdometryX()) >= targetDistanceX){
      forward = 0;
    }
    if(Math.abs(drivetrain.getOdometryY()) >= targetDistanceY){
      strafe = 0;
    }
    if(Math.abs(drivetrain.getOdometryZ()) >= targetAngleZ){
      rotation = 0;
    }

    //SmartDashboard.putNumber("forward", forward);
    //SmartDashboard.putNumber("strafe", strafe);
    //SmartDashboard.putNumber("rotation", rotation);

    //SmartDashboard.putNumber("Target Forward", targetForward);

    //SmartDashboard.putNumber("auto odometry X", drivetrain.getOdometryX());
    //SmartDashboard.putNumber("auto odometry Y", drivetrain.getOdometryY());
    //SmartDashboard.putNumber("auto odometry Z", drivetrain.getOdometryZ());

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

    //Make the robot finish its move based on odometry position. This means that unless you reset the gyro in-between moves, you are moving relative to the field not the robot
    //Basically just make sure that your new targets take this into account, or just reset the odometry in-between moves
    if(Math.abs(drivetrain.getOdometryX()) >= targetDistanceX && Math.abs(drivetrain.getOdometryY()) >= targetDistanceY && Math.abs(drivetrain.getOdometryZ()) >= targetAngleZ){
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, 0);

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, Math.atan2(B, C)*(180/Math.PI), 0);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, Math.atan2(B, D)*(180/Math.PI), 0);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, Math.atan2(A, C)*(180/Math.PI), 0);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, Math.atan2(A, D)*(180/Math.PI), 0);

      isFinished = true;
    }
    else{
      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, Math.atan2(B, C)*(180/Math.PI), 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, Math.atan2(B, D)*(180/Math.PI), 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, Math.atan2(A, C)*(180/Math.PI), 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, Math.atan2(A, D)*(180/Math.PI), 1);

      //Set speeds for modules
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, -frontLeftSpeed);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, -frontRightSpeed);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, -rearLeftSpeed);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, -rearRightSpeed);

      isFinished = false;

    }

  }  

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
