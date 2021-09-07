// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OIConstants {
        // The USB port of the driver joystick / controller.
        public static final int DriverControllerPort = 0;
    }


    public static final class SwerveConstants {
        // TODO: Figure out the max speed of the robot.
        // The maximum speed of the robot.
        public static final double MaximumSpeedMetersPerSecond = 0;

        // TODO: Figure out the left-right wheel distance.
        // The distance from the left to the right WHEEL centers.
        public static final double RobotWidthMeters = 0;

        // TODO: Figure out the front-back wheel distance.
        // The distance from the front the to back WHEEL centers.
        public static final double RobotLengthMeters = 0;

        // Defines the bounding box of the robot wheels.
        public static final SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(
                new Translation2d(RobotLengthMeters / 2, RobotWidthMeters / 2),
                new Translation2d(RobotLengthMeters / 2, -RobotWidthMeters / 2),
                new Translation2d(-RobotLengthMeters / 2, RobotWidthMeters / 2),
                new Translation2d(-RobotLengthMeters / 2, -RobotWidthMeters / 2));
    }

    public static final class SwerveModuleConstants {
        // The maximum turn wheel (not robot turn) speed in radians per second.
        public static final double maxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        // The maximum turn wheel (not robot turn) acceleration in radians per second^2.
        public static final double maxAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        // TODO: Figure out the drive motor's p-value
        // The P constant for the drive motor.
        public static final double driveMotorPValue = 0;

        // TODO: Figure out the turn motor's p-value.
        // The P constant for the turn motor.
        public static final double turnMotorPValue = 0;

        // TODO: Figure out the turn encoder distance per pulse.
        // The angular distance the turn encoder measures during a single pulse.
        // It is the whole rotation (2 * pi) divided by the resolution
        // of the encoder.
        public static final double turnEncoderDistancePerPulse = 0;

        // TODO: Figure out the drive wheel radius in meters.
        // The radius of the drive wheel in meters.
        public static final double driveWheelRadiusMeters = 0;

        // TODO: Figure out the drive wheel ratio.
        // The gear ratio from the output motor to the drive wheel.
        public static final double driveWheelGearRatio = 0;

        // By default, the Spark Max Encoder measures position in revolutions.
        // We want to change it to meters, since that measures distance.
        // We know that one revolution moves 2*pi radians. We multiply that
        // by the gearing from the motor to the wheel. Finally, we multiply that
        // by the radius of the wheel to find the distance moved.
        public static final double driveEncoderPositionConversionFactor = 2 * Math.PI * driveWheelGearRatio
                * driveWheelRadiusMeters;

        // For the velocity, we perform a similar conversion going from RPM (rev/min)
        // to meters per second. The only difference is that we must also divide by 60
        // because there are 60 seconds in one minute.
        public static final double driveEncoderVelocityConversionFactor = driveEncoderPositionConversionFactor / 60;
    }

    /**
     * These constants define the drive motors.
     */
    public static final class SwerveDriveMotorConstants {
        // TODO: Set the ports.
        public static final int FrontLeftPort = 1;
        public static final int FrontRightPort = 2;
        public static final int RearLeftPort = 3;
        public static final int RearRightPort = 4;
    }

    /**
     * These constants define the turn motors.
     */
    public static final class SwerveTurnMotorConstants {
        // TODO: Set the ports.
        public static final int FrontLeftPort = 1;
        public static final int FrontRightPort = 2;
        public static final int RearLeftPort = 3;
        public static final int RearRightPort = 4;
    }

    /**
     * These constants define the turn encoders.
     */
    public static final class SwerveTurnEncoderConstants {
        // TODO: Set the ports.
        public static final int[] FrontLeftPorts = new int[] { 0, 1 };
        public static final int[] FrontRightPorts = new int[] { 2, 3 };
        public static final int[] RearLeftPorts = new int[] { 4, 5 };
        public static final int[] RearRightPorts = new int[] { 6, 7 };
    }

    /**
     * These constants define if the drive motors are reversed.
     */
    public static final class SwerveDriveReversedConstants {
        // TODO: Figure out which drive motors should be reversed, if any.
        public static final boolean FrontLeft = false;
        public static final boolean FrontRight = false;
        public static final boolean RearLeft = false;
        public static final boolean RearRight = false;
    }

    /**
     * These constants define if the turn motors are reversed.
     */
    public static final class SwerveTurnReversedConstants {
        // TODO: Figure out which turn motors should be reverse, if any.
        public static final boolean FrontLeft = false;
        public static final boolean FrontRight = false;
        public static final boolean RearLeft = false;
        public static final boolean RearRight = false;
    }

}
