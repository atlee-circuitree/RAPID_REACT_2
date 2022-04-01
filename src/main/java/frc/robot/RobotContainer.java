// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import javax.crypto.SealedObject;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.commands.AutoDriveByMeters;
import frc.robot.commands.ControlClimbPistons;
import frc.robot.commands.DriveWithXbox;
import frc.robot.commands.KickoutFeeder;
import frc.robot.commands.RecalibrateModules;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunFeederAuto;
import frc.robot.commands.RunHook;
import frc.robot.commands.ShooterInAuto;
import frc.robot.commands.SmartDashboardCommand;
import frc.robot.commands.TurretAndShoot;
import frc.robot.commands.TurretRotateAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final Pneumatics pneumatics;
  private final FeederSubsystem feeder;
  private final LimeLightSubsystem limelight;
  private final TurretSubsystem turret;  
  
  //Regular Commands
  private final DriveWithXbox driveWithXbox;
  private final SmartDashboardCommand smartDashboardCommand;
  private final PerpetualCommand DWX_SDC_TUR;
  private final RecalibrateModules recalibrateModules;
  private final TurretAndShoot turretAndShoot;
   
  //Command Groups

  public Command feederCommand(double speed) {
    Command feedCommand = new RunFeeder(speed, feeder, pneumatics);
    return feedCommand;
  }

  public Command hookCommand(double speed) {
    Command hookCommand = new RunHook(speed, pneumatics);
    return hookCommand;
  }

  public Command climbCommand(boolean up) {
    Command hookCommand = new ControlClimbPistons(up, pneumatics);
    return hookCommand;
  }

  public Command WaitCommand(double timeout) {
    Command m_waitCommand = new WaitCommand(timeout);
    return m_waitCommand;
  }

  public Command adaptiveAutoShootCommand() {
    //Limelight stuff
    double distance = limelight.getDistanceToTarget();
 
    int roundedDistance = (int) Math.round(distance * 10);
    
    LaunchVelocity[] launchVelocityArray = turret.getDistanceToVelocityArray();

    double velocity = launchVelocityArray[roundedDistance].topMotorVelocity;

    double bottomVelocity = launchVelocityArray[roundedDistance].bottomMotorVelocity;
    Command m_shootCommand = new ShooterInAuto(velocity, bottomVelocity, turret, pneumatics, limelight, feeder);
    return m_shootCommand;
  }
  public Command AutoDriveCommand(double forward, double strafe, double rotation, double distanceMeters){
    Command m_AutoDriveByMeters = new AutoDriveByMeters(drivetrain, forward, strafe, rotation, distanceMeters);
    return m_AutoDriveByMeters;
  } 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
 
    //subsystems
    drivetrain = new Drivetrain();
    pneumatics = new Pneumatics();
    feeder = new FeederSubsystem();
    limelight = new LimeLightSubsystem();
    turret = new TurretSubsystem();

    //Teleop commands
    driveWithXbox = new DriveWithXbox(drivetrain, xbox, false);
    driveWithXbox.addRequirements(drivetrain);

    smartDashboardCommand = new SmartDashboardCommand();

    turretAndShoot = new TurretAndShoot(turret, pneumatics, limelight, xbox2);
 
    configureButtonBindings();

    limelight.EnableLED();

    //Other Setup

    recalibrateModules = new RecalibrateModules(drivetrain, xbox);

    DWX_SDC_TUR = new PerpetualCommand(driveWithXbox.alongWith(smartDashboardCommand).alongWith(turretAndShoot));
    
    //drivetrain.setDefaultCommand(recalibrateModules);
    drivetrain.setDefaultCommand(DWX_SDC_TUR);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //P1 BUTTONS
    JoystickButton DriverA = new JoystickButton(xbox, XboxController.Button.kA.value);
    JoystickButton DriverB = new JoystickButton(xbox, XboxController.Button.kB.value);
    
    JoystickButton DriverLB = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
    JoystickButton DriverRB = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);

    BooleanSupplier isDriverLTPressed = new BooleanSupplier() {
      
      @Override
      public boolean getAsBoolean() {
        if(xbox.getLeftTriggerAxis() > 0.1){
          return true;
        }
        else{
          return false;
        }
      }

    };
    BooleanSupplier isDriverRTPressed = new BooleanSupplier() {
      
      @Override
      public boolean getAsBoolean() {
        if(xbox.getRightTriggerAxis() > 0.1){
          return true;
        }
        else{
          return false;
        }
      }

    };

    Trigger DriverLT = new Trigger(isDriverLTPressed);
    Trigger DriverRT = new Trigger(isDriverRTPressed);
    
    DriverLT.whileActiveContinuous(feederCommand(-.72));
    DriverRT.whileActiveContinuous(feederCommand(.72));

    DriverLB.whileHeld(hookCommand(-.8));
    DriverRB.whileHeld(hookCommand(.8));

    //P2 BUTTONS
    JoystickButton Driver2A = new JoystickButton(xbox2, XboxController.Button.kA.value);
    JoystickButton Driver2B = new JoystickButton(xbox2, XboxController.Button.kB.value);
    JoystickButton Driver2Y = new JoystickButton(xbox2, XboxController.Button.kY.value);
    JoystickButton Driver2X = new JoystickButton(xbox2, XboxController.Button.kX.value);
    JoystickButton Driver2LB = new JoystickButton(xbox2, 5);
    JoystickButton Driver2RB = new JoystickButton(xbox2, 6);

    //7600 works from close safe zone
    //Driver2A.whenPressed(SimpleShootCommand(7600));
    //Driver2B.whenPressed(SimpleShootCommand(8400));
    //9200 works from far safe zone
    //Driver2Y.whenPressed(SimpleShootCommand(9200));
    //Driver2X.whenPressed(SimpleShootCommand(4000));

    //FIGHTSTICK BUTTONS
    JoystickButton FightstickShare = new JoystickButton(fightstick, 7);
    JoystickButton FightstickOption = new JoystickButton(fightstick, 8);
    JoystickButton FightstickB = new JoystickButton(fightstick, 2);
    JoystickButton FightstickY = new JoystickButton(fightstick, 4);
    JoystickButton FightstickL3 = new JoystickButton(fightstick, 9);
    JoystickButton FightstickR3 = new JoystickButton(fightstick, 10);
   
    FightstickShare.whenPressed(new KickoutFeeder(false, pneumatics, .1));
    FightstickOption.whenPressed(new KickoutFeeder(true, pneumatics, .1));
    FightstickL3.whenHeld(climbCommand(true));
    FightstickR3.whenHeld(climbCommand(false));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                2, 1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.driveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Starting Position
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior Waypoints
            List.of(new Translation2d(1, 0)),
            // Ending Position
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.kPAutoThetaController, 0, 0, Constants.thetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            drivetrain::getPose, // Functional interface to feed supplier
            Constants.driveKinematics,

            // Position controllers
            new PIDController(Constants.kPXAutoController, 0, 0),
            new PIDController(Constants.kPYAutoController, 0, 0),
            thetaController,
            drivetrain::setSwerveModuleStates,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.zeroNavXYaw();
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());


    SequentialCommandGroup autoTest = new SequentialCommandGroup(
    new ResetGyro(drivetrain),
    new KickoutFeeder(false, pneumatics, 1),
    new RunFeederAuto(.8, feeder, pneumatics, .1),
    swerveControllerCommand.andThen(() -> drivetrain.killAllModulesNonLinear()),
    new TurretRotateAuto(turret, limelight, 2),
    adaptiveAutoShootCommand(),
    adaptiveAutoShootCommand()
    );

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> drivetrain.driveAllModulesNonLinear(0));

    return autoTest;
    //return AutoDriveCommand(0.5, 0.5, 0, 5);
  }
}
