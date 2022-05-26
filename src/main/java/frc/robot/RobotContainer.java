// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithXbox;
import frc.robot.commands.RecalibrateModules;
import frc.robot.commands.SmartDashboardCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pathweaver;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //Controllers
  public static XboxController xbox = new XboxController(0);;
  public static XboxController xbox2 = new XboxController(1);
  public static Joystick fightstick = new Joystick(2);
  
  //Subsystems
  private final Drivetrain drivetrain;
  private final Pathweaver pathweaver;
  
  //Regular Commands
  private final DriveWithXbox driveWithXbox;
  private final SmartDashboardCommand smartDashboardCommand;
  private final PerpetualCommand DWX_With_SDC;
  private final RecalibrateModules recalibrateModules;
   


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
 
    //subsystems
    drivetrain = new Drivetrain();
    pathweaver = new Pathweaver();

    //Teleop commands
    driveWithXbox = new DriveWithXbox(drivetrain, xbox, false);
    driveWithXbox.addRequirements(drivetrain);

    smartDashboardCommand = new SmartDashboardCommand();
    
    configureButtonBindings();

    //Other Setup

    recalibrateModules = new RecalibrateModules(drivetrain, xbox);

    DWX_With_SDC = new PerpetualCommand(driveWithXbox.alongWith(smartDashboardCommand));
    
    //drivetrain.setDefaultCommand(recalibrateModules);
    drivetrain.setDefaultCommand(DWX_With_SDC);

  }


  private void configureButtonBindings() {

  }


  //Note: No auto implementation for now, but you can look at the actual command
  public Command getAutonomousCommand() {
    
    return null;
    
  }
}
