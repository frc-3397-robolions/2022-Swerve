// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Autonomous;
import frc.robot.commands.SpinCommand;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Constants c;

  private SwerveModule backRight;      
  private SwerveModule backLeft;       
  private SwerveModule frontRight;
  private SwerveModule frontLeft;      

  private DriveTrain swerveDrive; 
  
  private XboxController driveController;
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    c = new Constants();
    backRight = new SwerveModule (c.BACK_RIGHT_ANGLE_ID, c.BACK_RIGHT_SPEED_ID);
    backLeft = new SwerveModule (c.BACK_LEFT_ANGLE_ID, c.BACK_LEFT_SPEED_ID);
    frontRight = new SwerveModule (c.FRONT_RIGHT_ANGLE_ID, c.FRONT_RIGHT_SPEED_ID);
    frontLeft = new SwerveModule (c.FRONT_LEFT_ANGLE_ID, c.FRONT_LEFT_SPEED_ID);
    swerveDrive = new DriveTrain (backRight, backLeft, frontRight, frontLeft); 
    driveController= new XboxController(Constants.XB_CONTROLLER_PORT);
    swerveDrive.setDefaultCommand(new SwerveDrive(swerveDrive, driveController));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveController, XboxController.Button.kX.value).whenHeld(new SpinCommand(swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Autonomous();
  }
}
