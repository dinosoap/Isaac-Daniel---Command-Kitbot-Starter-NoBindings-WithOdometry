// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class TurnToHeadingCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private final DriveSubsystem m_drivetrain;
  private final double m_targetAngle;
  private final double turnConst = 0.01;
  private final double maxTurn = 0.5;
  private final double minTurn = 0.1;
  private final double m_ToleranceDegrees = 1;

  public TurnToHeadingCommand(DriveSubsystem drivetrain, double targetangle) {
    m_drivetrain = drivetrain;
    m_targetAngle = targetangle;

    addRequirements(m_drivetrain);
  }

  
  @Override
  public void execute(){
    double CurrentHeading = m_drivetrain.getPose().getRotation().getDegrees();
    double HeadingError = MathUtil.inputModulus(m_targetAngle - CurrentHeading, -180, 180 );
    double TurnSpeed = HeadingError * turnConst;
    SmartDashboard.putNumber("CurrentHeading", CurrentHeading);
    SmartDashboard.putNumber("headingerror", HeadingError);
    double turnSpeed;
    if (TurnSpeed > 0) {
      turnSpeed = MathUtil.clamp(TurnSpeed, minTurn, maxTurn);
    }
    else {
      turnSpeed = MathUtil.clamp(TurnSpeed, -maxTurn, -minTurn);
    }
    SmartDashboard.putNumber("TurnSpeed", turnSpeed);
    m_drivetrain.m_differentialDrive.arcadeDrive( 0, turnSpeed);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDriveCommand(() -> 0, () -> 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double CurrentHeading = m_drivetrain.getPose().getRotation().getDegrees();
    double HeadingError = MathUtil.inputModulus(m_targetAngle - CurrentHeading, -180, 180 );
    return Math.abs(HeadingError) <= m_ToleranceDegrees;
  }


  // public static void TurnToHeadingCommand(DriveSubsystem m_driveSubsystem, double d) {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'TurnToHeadingCommand'");
  // }
}
