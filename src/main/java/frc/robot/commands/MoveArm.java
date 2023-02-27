// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm arm;
  private XboxController xbC;
  private Joystick pad;
  private double bottomAngle;
  private double topAngle;
  private double x=0;
  private double y=0;
  private double a2=Constants.ARM_JOINT_2_LENGTH;
  private double a1=Constants.ARM_JOINT_1_LENGTH;
  public MoveArm(Arm sub, XboxController xbC,Joystick pad) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
    arm=sub;
    this.xbC=xbC;
    this.pad=pad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    y=clamp(y, -4.2,0);
    topAngle=Math.acos(x*x+y*y-a1*a1-a2*a2/(2*a1*a2));
    bottomAngle=Math.atan(x/y)+Math.atan(a2*Math.sin(topAngle)/(a1+a2*Math.cos(topAngle)));
    topAngle=clamp(topAngle,-180,180);
    bottomAngle=clamp(bottomAngle,0,300)
    SmartDashboard.putNumber("bottomAngle", bottomAngle);
    SmartDashboard.putNumber("topAngle", topAngle);
    arm.setBottomAngle(bottomAngle);
    arm.setTopAngle(topAngle);

    if(xbC.getBackButton()){
      arm.zeroBottomEncoder();
    }
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
