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
    if (xbC.getLeftBumper()){
      bottomAngle+=0.3;
    }
    else if (xbC.getRightBumper()){
      bottomAngle-=0.3;
    }
    //Folded Position
    if(pad.getRawButton(0)){
      bottomAngle=0;
      topAngle=0;
    }

    arm.setBottomAngle(bottomAngle);
    arm.setTopAngle(topAngle);

    SmartDashboard.putNumber("bottomAngle", bottomAngle);
    SmartDashboard.putNumber("topAngle", topAngle);

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
