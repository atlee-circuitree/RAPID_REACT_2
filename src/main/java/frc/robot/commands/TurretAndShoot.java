// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LaunchVelocity;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAndShoot extends CommandBase {

  //This is the shooting command to end all shooting commands
  //This serves three functions
  //1. Spin the motors up to speed while holding a button (preset values, limelight distance table values, shuffleboard values)
  //2. Control the pnuematics based on motor velocity and weather the button is held to do multiple shots
  //3. Adjust the turret based on the limelight

  //Some notes for debugging
  //targetTopVelocity and targetBottomVelocity are the velocity values the motor trys to achieve
  
  private final TurretSubsystem turret;
  private final Pneumatics pneumatic;
  private final LimeLightSubsystem limelight;

  private double adaptiveTopVelocity = 0;
  private double adaptiveBottomVelocity = 0;

  private double targetTopVelocity;
  private double targetBottomVelocity;

  private boolean InShot = false;
 
  private XboxController xboxController;

  private double distance;

  private double timeout = .3;

  Timer pneumaticTime = new Timer();
  Timer timeoutTime = new Timer();

  public TurretAndShoot(TurretSubsystem ts, Pneumatics ps, LimeLightSubsystem ls, XboxController xbox) {
 
    turret = ts;
    pneumatic = ps;
    limelight = ls;
    xboxController = xbox;
    addRequirements(turret, pneumatic, limelight);
    
  }
 
  @Override
  public void initialize() {
  
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //Constantly update values for the adaptive shot
    if(!xboxController.getLeftBumper()){
      distance = limelight.getDistanceToTarget();
 
      int roundedDistance = (int) Math.round(distance * 10);

      if(roundedDistance >= 60) {

        roundedDistance = 60;

      }
    
      LaunchVelocity[] launchVelocityArray = turret.getDistanceToVelocityArray();

      adaptiveTopVelocity = launchVelocityArray[roundedDistance].topMotorVelocity;

      adaptiveBottomVelocity = launchVelocityArray[roundedDistance].bottomMotorVelocity;
    }
    else{
      //do nothing
    }
    //SmartDashboard.putNumber("Limelight Rounded Distance", roundedDistance);
    SmartDashboard.putNumber("Adaptive Top Velocity", adaptiveTopVelocity);
    SmartDashboard.putNumber("Adaptive Bottom Velocity", adaptiveBottomVelocity);

    //Change velocity based on what buttons are held
    //A = Adaptive, B = Medium, Y = High, X = Low, Back = Shuffleboard
    if (xboxController.getAButton() == true && InShot == false) {

    turret.runTurretWithVelocity(adaptiveTopVelocity, adaptiveBottomVelocity);

    targetTopVelocity = adaptiveTopVelocity;
    targetBottomVelocity = adaptiveBottomVelocity;
 
    } else if (xboxController.getBButton() == true && InShot == false) {

    turret.runTurretWithVelocity(Constants.mediumVelocityTop, Constants.mediumVelocityBottom);

    targetTopVelocity = Constants.mediumVelocityTop;
    targetBottomVelocity = Constants.mediumVelocityBottom;

    } else if (xboxController.getYButton() == true && InShot == false) {

    turret.runTurretWithVelocity(Constants.highVelocityTop, Constants.highVelocityBottom);

    targetTopVelocity = Constants.highVelocityTop;
    targetBottomVelocity = Constants.highVelocityBottom;

    } else if (xboxController.getXButton() == true && InShot == false) {

    turret.runTurretWithVelocity(Constants.lowVelocityTop, Constants.lowVelocityBottom);

    targetTopVelocity = Constants.lowVelocityTop;
    targetBottomVelocity = Constants.lowVelocityBottom;
 
    } else if (xboxController.getLeftBumper() == true && InShot == false) {

    //turret.runTurretWithVelocity(turret.getShuffleTopMotor(), turret.getShuffleBottomMotor());

    //targetTopVelocity = turret.getShuffleTopMotor();
    //targetBottomVelocity = turret.getShuffleBottomMotor();
    turret.runTurretWithVelocity(adaptiveTopVelocity, adaptiveBottomVelocity);

    targetTopVelocity = adaptiveTopVelocity;
    targetBottomVelocity = adaptiveBottomVelocity;

    } else if (InShot == false) {

    turret.runTurretWithVelocity(0, 0);

    targetTopVelocity = 0;
    targetBottomVelocity = 0;

    }

    //Start the shot timer and freeze velocity adjustments 
    if (xboxController.getRightBumper() == true && InShot == false) {

    InShot = true;
    pneumaticTime.start();
    timeoutTime.start();
    pneumaticTime.reset();
    timeoutTime.reset();

    }

    //Pneumatic logic
    
    if (turret.checkShootVelocity(targetTopVelocity, targetBottomVelocity, timeoutTime, timeout) == false && InShot == true) {

    pneumaticTime.reset();

    System.out.println("Starting " + timeoutTime.get());

    } else if (pneumaticTime.get() < .7 && InShot == true) {

    pneumatic.shooterUp();

    System.out.println("Going up " + pneumaticTime.get());

    } else if (pneumaticTime.get() < 1.15 && InShot == true) {

    pneumatic.shooterDown();

    System.out.println("Going down " + pneumaticTime.get());

    } else if (pneumaticTime.get() >= 1.15 && xboxController.getRightBumper() == true && InShot == true) {

    pneumatic.shooterDown();

    pneumaticTime.reset();
    timeoutTime.reset();
 
    System.out.println("Reseting " + pneumaticTime.get());
  
    } else if (InShot == true) {

    pneumatic.shooterDown();

    pneumaticTime.reset();
    timeoutTime.reset();
    pneumaticTime.stop();
    timeoutTime.stop();

    InShot = false;
 
    System.out.println("Done " + pneumaticTime.get());

    }
    

    //Turret logic
    if (xboxController.getLeftTriggerAxis() > 0.2) {

      turret.turnTurret(xboxController.getLeftX() / 4);

    } else if (limelight.HorizontalOffset() > .2 && limelight.HasValidTarget() == true) {

      turret.turnTurret(limelight.HorizontalOffset() / 20);

    } else if (limelight.HorizontalOffset() < -.2 && limelight.HasValidTarget() == true) {

      turret.turnTurret(limelight.HorizontalOffset() / 20);

    } else {

      turret.turnTurret(0);

    }

    //Reset Turret Encoder
    if (xboxController.getStartButton() == true) {

      turret.resetEncoder();
  
    }


  }

  @Override
  public void end(boolean interrupted) {
     
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
