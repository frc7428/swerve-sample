// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveDriveMotorConstants;
import frc.robot.Constants.SwerveDriveReversedConstants;
import frc.robot.Constants.SwerveTurnEncoderConstants;
import frc.robot.Constants.SwerveTurnMotorConstants;
import frc.robot.Constants.SwerveTurnReversedConstants;

public class DriveSubsystem extends SubsystemBase {
  // Define the four modules.
  private final SwerveModule mFrontLeft = new SwerveModule(SwerveDriveMotorConstants.FrontLeftPort,
      SwerveTurnMotorConstants.FrontLeftPort, SwerveTurnEncoderConstants.FrontLeftPorts,
      SwerveDriveReversedConstants.FrontLeft, SwerveTurnReversedConstants.FrontLeft);

  private final SwerveModule mFrontRight = new SwerveModule(SwerveDriveMotorConstants.FrontRightPort,
      SwerveTurnMotorConstants.FrontRightPort, SwerveTurnEncoderConstants.FrontRightPorts,
      SwerveDriveReversedConstants.FrontRight, SwerveTurnReversedConstants.FrontRight);

  private final SwerveModule mRearLeft = new SwerveModule(SwerveDriveMotorConstants.RearLeftPort,
      SwerveTurnMotorConstants.RearLeftPort, SwerveTurnEncoderConstants.RearLeftPorts,
      SwerveDriveReversedConstants.RearLeft, SwerveTurnReversedConstants.RearLeft);

  private final SwerveModule mRearRight = new SwerveModule(SwerveDriveMotorConstants.RearRightPort,
      SwerveTurnMotorConstants.RearRightPort, SwerveTurnEncoderConstants.RearRightPorts,
      SwerveDriveReversedConstants.RearRight, SwerveTurnReversedConstants.RearRight);

  /**
   * Empty constructor.
   */
  public DriveSubsystem() {
  }

  public void drive(double forwardSpeed, double rightSpeed, double rotationSpeed) {
    // Compute the states for the four modules.
    SwerveModuleState[] swerveModuleStates = SwerveConstants.Kinematics
        .toSwerveModuleStates(new ChassisSpeeds(forwardSpeed, rightSpeed, rotationSpeed));
    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    // Normalize the speeds to the maximum output.
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, SwerveConstants.MaximumSpeedMetersPerSecond);

    // Set the speeds.
    // Index 0 - Front Left
    // Index 1 - Front Right
    // Index 2 - Rear Left
    // Index 3 - Rear Right
    mFrontLeft.setDesiredState(swerveModuleStates[0]);
    mFrontRight.setDesiredState(swerveModuleStates[1]);
    mRearLeft.setDesiredState(swerveModuleStates[2]);
    mRearRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
  }
}
