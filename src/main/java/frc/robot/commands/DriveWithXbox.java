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

public class DriveWithXbox extends CommandBase {

  private final Drivetrain drivetrain;
  private SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(0.8);
  private SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(0.8);
  private SlewRateLimiter slewRateLimiterZ = new SlewRateLimiter(0.8);
  
  private XboxController xbox;

  public boolean isTesting;

  private double forward;
  private double strafe;
  private double rotation;

  public static String driveWithXboxDashboard;
 
  public DriveWithXbox(Drivetrain dt, XboxController xboxController, boolean testing) {
    
    drivetrain = dt;
    xbox = xboxController;

    isTesting = testing;

    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    /*
    Holy cow this is going to be A LOT of code eventually...
    (Actually, it's pretty compact/efficient currently. I thought this was going to need a lot more code... - Simon 8/3/21)
    (You were a fool, past Simon. This is going to be a lot of code, and even more math - Simon 10/11/21)
    (This is getting out of hand *Screams in PID* - Simon 11/12/21)

    IF YOU DO WANT TO EDIT THIS COMMAND, BE SURE TO READ THE SWERVE PDFs
    (can be found on chief delphi, search for "4 wheel independent drive independent steering swerve", should be 1st 2 PDFs)

    Steps of what we need to do:
    1. Convert joystick X/Y values to degrees **Added compicated yet necessary math**   
    2. Modify that value by NavX position to do field orientation
    3. Feed final value to a rotateModules() function
    4. Get speed value from joystick
    5. Feed that to a driveMotors() function 
    6. Add in a rotateEntireRobot() function using the other joystick and hope it doesnt break anything 
    7. Debug the heck out of this command **Currently on this step - Simon**
    */

    //Define robot target vector variables (X,Y,Z respectively)  
    //double forward = -slewRateLimiterX.calculate(RobotContainer.xbox.getLeftY());
    //double strafe = -slewRateLimiterY.calculate(RobotContainer.xbox.getLeftX());
    //double rotation = -slewRateLimiterZ.calculate(RobotContainer.xbox.getRightX());

    /*

    if (isTesting = true && xbox.getAButton() == true) {

      forward = -1;
      strafe = 0;
      rotation = 0;

    } else if (isTesting = true && xbox.getBButton() == true) {
     
      forward = 0;
      strafe = 1;
      rotation = 0;

    } else if (isTesting = true && xbox.getXButton() == true) {

      forward = 0;
      strafe = -1;
      rotation = 0;

    } else if (isTesting = true && xbox.getYButton() == true) {

      forward = 1;
      strafe = 0;
      rotation = 0;

    } else if (isTesting = true) {

      forward =0;
      strafe = 0;
      rotation = 0;
  
    } else {

      forward = -xbox.getLeftY();
      strafe = -xbox.getLeftX();
      rotation = -xbox.getRightX();

    }

    */

    forward = -xbox.getLeftY() / 2;
    strafe = -xbox.getLeftX() / 2;
    rotation = xbox.getRightX() / 2;

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
    return false;
  }
}
