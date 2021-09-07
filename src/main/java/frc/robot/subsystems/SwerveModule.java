// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
    /**
     * These are the components tied to a Spark Max. If you're using a Spark Max,
     * you'll also likely use its special encoder and its special PID controller.
     */
    private final CANSparkMax mDriveMotor;
    private final CANEncoder mDriveEncoder;
    private final CANPIDController mDrivePIDController;

    /**
     * These are the components you'll use for a more standard motor controller. The
     * encoder and the PID controller come from WPI Lib, not from the Victor.
     */
    private final WPI_VictorSPX mTurnMotor;
    private final Encoder mTurnEncoder;
    private final ProfiledPIDController mTurnPIDController;

    /**
     * This is the constructor. It is used to set up the module.
     * 
     * @param driveMotorChannel    the ID of the Spark Max controlling the drive
     *                             motor
     * @param turnMotorChannel     the ID of the Victor SPX controlling the turn
     *                             motor
     * @param turnEncoderChannels  the two channels being used for the turn encoder
     *                             in the roboRIO
     * @param driveEncoderReversed whether the drive motor/encoder is backwards from
     *                             the default
     * @param turnEncoderReversed  whether the turn motor/encoder is backwards from
     *                             the default
     */
    public SwerveModule(int driveMotorChannel, int turnMotorChannel, int[] turnEncoderChannels,
            boolean driveEncoderReversed, boolean turnEncoderReversed) {
        /**
         * For the Spark Max, the Encoder and the PID Controller both come from the
         * motor controller, so we create it first, then we retrieve them from it.
         */
        mDriveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        mDriveEncoder = mDriveMotor.getEncoder();
        mDrivePIDController = mDriveMotor.getPIDController();

        /**
         * This is how you set the PID constants. If you don't want to use certain ones,
         * leave them as 0. For now, you probably only want to adjust the P value.
         */
        mDrivePIDController.setP(SwerveModuleConstants.driveMotorPValue);
        mDrivePIDController.setI(0);
        mDrivePIDController.setD(0);

        /**
         * Instead of using the internal revolutions and revolutions per minute (RPM),
         * we want to convert the position and velocity measurements into meters and
         * meters per second.
         */
        mDriveEncoder.setPositionConversionFactor(SwerveModuleConstants.driveEncoderPositionConversionFactor);
        mDriveEncoder.setVelocityConversionFactor(SwerveModuleConstants.driveEncoderVelocityConversionFactor);

        /**
         * For the Victor, you'll use the standard WPILib for the encoder and the PID
         * controller.
         * 
         * The first three values in the ProfiledPIDController are the P, I, and D
         * values. Just like before, leave them as 0 if you don't want to use them, but
         * you probably want to set only the P-value (the first one).
         * 
         * The TrapezoidProfile defines how the PID controller is going to work, using
         * pre-defined velocity and acceleration limitations. These values you might be
         * able to play with later.
         * 
         * These values restrict how quickly the turn motor can adjust the angle and how
         * quickly the turn motor can speed up or slow down its adjustment of the angle.
         */
        mTurnMotor = new WPI_VictorSPX(turnMotorChannel);
        mTurnEncoder = new Encoder(turnEncoderChannels[0], turnEncoderChannels[1]);
        mTurnPIDController = new ProfiledPIDController(SwerveModuleConstants.turnMotorPValue, 0, 0,
                new TrapezoidProfile.Constraints(SwerveModuleConstants.maxAngularSpeedRadiansPerSecond,
                        SwerveModuleConstants.maxAngularAccelerationRadiansPerSecondSquared));

        /**
         * Now we apply the settings of the turn encoder. This includes the angular
         * distance it moves in a single pulse as well as whether or not to reverse the
         * direction (clockwise vs. counterclockwise).
         */
        mTurnEncoder.setDistancePerPulse(SwerveModuleConstants.turnEncoderDistancePerPulse);
        mTurnEncoder.setReverseDirection(turnEncoderReversed);

        /**
         * Instead of using the default of 0 to 2*pi for a whole rotation, we want to
         * use the alternative of -pi to +pi. This will simplify the rotations.
         */
        mTurnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * This method will give the current state of the module
     * 
     * @return A state that indicates both the drive motor's speed and the turn
     *         motor's position.
     */
    public SwerveModuleState getState() {
        /**
         * The drive encoder's getVelocity should return the drive speed in
         * meters/second. The turn encoder's get should return the count of the turn
         * motor.
         */
        return new SwerveModuleState(mDriveEncoder.getVelocity(), new Rotation2d(mTurnEncoder.get()));
    }

    /**
     * Change the module's state (speed and direction).
     * 
     * @param desiredState A state that indicates the drive motor's desired speed
     *                     and the turn motor's desired position.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // The optimize method calculates the best equivalent state
        // Since multiple angles/directions of driving are equivalent.
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(mTurnEncoder.get()));

        // Extract the drive speed from the state.
        final double driveSpeed = state.speedMetersPerSecond;

        // Extract the turn encoder position and compute the right output for the
        // desired angle.
        final double turnAngle = mTurnPIDController.calculate(mTurnEncoder.get(), state.angle.getRadians());

        // Set the drive motor's PID controller's reference.
        mDrivePIDController.setReference(driveSpeed, ControlType.kVelocity);

        // Set the turn motor's output.
        mTurnMotor.set(turnAngle);
    }

}
