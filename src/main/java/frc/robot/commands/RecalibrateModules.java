// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RecalibrateModules extends CommandBase {

  private final Drivetrain drivetrain;
  private int moduleSelected = 1;
  private static XboxController xbox;


  public RecalibrateModules(Drivetrain subsystem, XboxController xbc) {
    
    drivetrain = subsystem;
    xbox = xbc;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed;

    speed = 0.06 * (xbox.getTriggerAxis(Hand.kLeft) - xbox.getTriggerAxis(Hand.kRight));
     
    if(xbox.getAButtonPressed()){
      moduleSelected++;
    }

    if(moduleSelected == 1){
      drivetrain.rotateMotor(Motors.FRONT_LEFT_ROT, speed);
    }
    else if(moduleSelected == 2){
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_ROT, speed);
    }
    else if(moduleSelected == 3){
      drivetrain.rotateMotor(Motors.REAR_LEFT_ROT, speed);
    }
    else if(moduleSelected == 4){
      drivetrain.rotateMotor(Motors.REAR_RIGHT_ROT, speed);
    }
    else{
      moduleSelected = 1;
    }

    SmartDashboard.putNumber("testSpeed", speed);
    SmartDashboard.putNumber("Module selected", moduleSelected);
    SmartDashboard.putNumber("FL Encoder", drivetrain.getRotEncoderValue(SwerveModule.FRONT_LEFT));
    SmartDashboard.putNumber("FR Encoder", drivetrain.getRotEncoderValue(SwerveModule.FRONT_RIGHT));
    SmartDashboard.putNumber("RL Encoder", drivetrain.getRotEncoderValue(SwerveModule.REAR_LEFT));
    SmartDashboard.putNumber("RR Encoder", drivetrain.getRotEncoderValue(SwerveModule.REAR_RIGHT));

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
