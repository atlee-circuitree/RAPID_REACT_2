// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  A semi-quick explanation on how this file works for the person that has to debug this:

  There are global (public static) strings in each of the files we need variables from, they should be named ________Dashboard
  They are there to "package" all of the local (private) variables in that certain subsystem/command to "send" to this file
  I have structured them in a very specific way in order to "unpack" them in this file and push them to SmartDashboard
  
  Backslashes separate variable names from their values, and semicolons separate pairs of name/values
  Like this: name1/value1;name2/value2;name3/value3

  How this code actually breaks the strings apart relies on the String.split() function, which breaks up a string around a certain character
  
  Step 1: Get global Dashboard string from subsystem/command and split it around the semicolon
  (string is now an array that looks like this [[name1/value1],[name2/value2],[name3/value3]])
  
  Step 2: For each element in that new array, split it around the backslash
  (first loop = [name1,value1]    second loop = [name2,value2]    third loop = [name3,value3])
  
  Step 3: Push the 1st element in the double split string as the variable name, and the second one as the variable value

*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;


public class SmartDashboardCommand extends CommandBase {

  
  public SmartDashboardCommand() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    //LOCAL VARS FROM COMMANDS


    //DriveWithXbox display local vars  
    
    String[] splitStringArrayDWX = DriveWithXbox.driveWithXboxDashboard.split(";");

    for(int i = 0; i <= splitStringArrayDWX.length-1; i++){

      String[] splitSplitStringArrayDWX = splitStringArrayDWX[i].split("/");
      SmartDashboard.putString(splitSplitStringArrayDWX[0], splitSplitStringArrayDWX[1]);

    }

    //VARS FROM ROBOTCONTAINER AND DRIVETRAIN

    //Robotcontainer vars
    SmartDashboard.putNumber("Xbox left X value", RobotContainer.xbox.getX(Hand.kLeft));
    SmartDashboard.putNumber("Xbox left Y value", RobotContainer.xbox.getY(Hand.kLeft));
    
    //Drivetrain display local vars
    String[] splitStringArrayDVT = Drivetrain.drivetrainDashboard.split(";");
    SmartDashboard.putNumber("DVT String Length", splitStringArrayDVT.length);
    
    for(int i = 0; i <= splitStringArrayDVT.length-1; i++){

      String[] splitSplitStringArrayDVT = splitStringArrayDVT[i].split("/");
      SmartDashboard.putString(splitSplitStringArrayDVT[0], splitSplitStringArrayDVT[1]);

    }

  }  

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
