// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Create the drive subsystem.
  private final DriveSubsystem mDrive = new DriveSubsystem();

  // Create the drive joystick.
  Joystick mDriveController = new Joystick(OIConstants.DriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();

    // Set the default command.
    // Note that you *may* have to make some of these negative, but maybe not.
    mDrive.setDefaultCommand(new RunCommand(
        () -> mDrive.drive(mDriveController.getY(), mDriveController.getX(), mDriveController.getZ()), mDrive));
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
