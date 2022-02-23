// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TestRotateModules extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drivetrain drivetrain;


  public TestRotateModules(Drivetrain subsystem) {
    
    drivetrain = subsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.zeroNavXYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Define robot target vector variables (X,Y,Z respectively)  
    double forward = 1;
    double strafe = 0;
    double rotation = 0;

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
    if((forward == 0 && strafe == 0 && rotation == 0)){
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, 0);

      drivetrain.rotateModuleNonLinear(SwerveModule.FRONT_LEFT, Math.atan2(B, D)*(180/Math.PI), 0);
      drivetrain.rotateModuleNonLinear(SwerveModule.FRONT_RIGHT, Math.atan2(B, C)*(180/Math.PI), 0);
      drivetrain.rotateModuleNonLinear(SwerveModule.REAR_LEFT, Math.atan2(A, D)*(180/Math.PI), 0);
      drivetrain.rotateModuleNonLinear(SwerveModule.REAR_RIGHT, Math.atan2(A, C)*(180/Math.PI), 0);
    }
    else{
      //Set angles for modules (change speed mod later if needed)
      drivetrain.rotateModuleNonLinear(SwerveModule.FRONT_LEFT, Math.atan2(B, D)*(180/Math.PI), 1);
      drivetrain.rotateModuleNonLinear(SwerveModule.FRONT_RIGHT, Math.atan2(B, C)*(180/Math.PI), 1);
      drivetrain.rotateModuleNonLinear(SwerveModule.REAR_LEFT, Math.atan2(A, D)*(180/Math.PI), 1);
      drivetrain.rotateModuleNonLinear(SwerveModule.REAR_RIGHT, Math.atan2(A, C)*(180/Math.PI), 1);

      //Set speeds for modules
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, frontLeftSpeed*0);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, frontRightSpeed*0);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, rearLeftSpeed*0);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, rearRightSpeed*0);
    }

    SmartDashboard.putNumber("DEBUG FL ENCODER", drivetrain.getRotEncoderValue(SwerveModule.FRONT_LEFT));
    SmartDashboard.putNumber("DEBUG FR ENCODER", drivetrain.getRotEncoderValue(SwerveModule.FRONT_RIGHT));
    SmartDashboard.putNumber("DEBUG RL ENCODER", drivetrain.getRotEncoderValue(SwerveModule.REAR_LEFT));
    SmartDashboard.putNumber("DEBUG RR ENCODER", drivetrain.getRotEncoderValue(SwerveModule.REAR_RIGHT));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
