// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class TestDriveCommand extends CommandBase {

  private final Drivetrain drivetrain;
  private double speed = 0;
  private double rotspeed = 0;
  
  public TestDriveCommand(Drivetrain dt) {
    
    drivetrain = dt;
    
    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    speed = RobotContainer.xbox.getY(Hand.kLeft);
    rotspeed = RobotContainer.xbox.getX(Hand.kRight);

    drivetrain.driveAllModulesNonLinear(speed);
    drivetrain.rotateAllModulesNonLinear(90, rotspeed);

    System.out.println(drivetrain.getRotEncoderValue(SwerveModule.FRONT_LEFT));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
