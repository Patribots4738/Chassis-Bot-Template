// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package hardware;

import calc.Constants.*;
import calc.PhotonCameraPose;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;


/**
 * Represents a swerve drive style drivetrain.
 */
public class Swerve {

    private static final PhotonCameraPose photonCameraPose = new PhotonCameraPose();
    SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
    SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
            DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
            DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);
    private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
            DriveConstants.BACK_LEFT_DRIVING_CAN_ID,
            DriveConstants.BACK_LEFT_TURNING_CAN_ID,
            DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);
    private final MAXSwerveModule m_backRight = new MAXSwerveModule(
            DriveConstants.BACK_RIGHT_DRIVING_CAN_ID,
            DriveConstants.BACK_RIGHT_TURNING_CAN_ID,
            DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    private final MAXSwerveModule[] MAXSwerveModules = new MAXSwerveModule[]{
            m_frontLeft,
            m_frontRight,
            m_backLeft,
            m_backRight
    };

    /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
    below are robot specific, and should be tuned. */
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    DriveConstants.DRIVE_KINEMATICS,
                    getGyroAngle(),
                    getModulePositions(),
                    new Pose2d(),
                    // These standard deviations are the default values, (the higher these values are, the less you trust the measurements)
                    // even if these params removed,
                    // the standard deviations will be set to these values
                    VecBuilder.fill(0.1, 0.1, 0.1), // State measurement standard deviations
                    VecBuilder.fill(0.9, 0.9, 0.9)); // Vision measurement standard deviations
    private final Field2d field = new Field2d();
    private final MAXSwerveModule[] swerveModules = new MAXSwerveModule[]{
            m_frontLeft,
            m_frontRight,
            m_backLeft,
            m_backRight
    };


    public Swerve() {
        resetEncoders();
        // TODO: add methods for gyro and make a new class for them
        zeroHeading();
        setBreakMode(true);
        SmartDashboard.putData("Field", field);
    }

    public void setBreakMode(boolean brakeMode) {
        for (MAXSwerveModule swerveModule : swerveModules) {
            // if brakeMode is true, setBrakeMode will be called, otherwise setCoastMode will be called with a one line if statement
            if (brakeMode) { swerveModule.setBrakeMode(); }
            else { swerveModule.setCoastMode(); }
        }
    }

    public void periodic() {
        updateOdometry();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotation      Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param rateLimited   Whether the robot should be rate limited.
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative, boolean rateLimited) {
        SwerveModuleState[] swerveModuleStates;
        if (rateLimited) {

            swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                    fieldRelative
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeedLimiter.calculate(xSpeed),
                            ySpeedLimiter.calculate(ySpeed),
                            rotationLimiter.calculate(rotation),
                            getPose().getRotation())
                            : new ChassisSpeeds(
                            xSpeedLimiter.calculate(xSpeed) * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                            ySpeedLimiter.calculate(ySpeed) * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                            rotationLimiter.calculate(rotation) * DriveConstants.MAX_ANGULAR_SPEED));

        } else {
            swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                    fieldRelative
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeed,
                            ySpeed,
                            rotation,
                            getPose().getRotation())
                            : new ChassisSpeeds(
                            xSpeed,
                            ySpeed,
                            rotation));
        }
        setModuleStates(swerveModuleStates);
    }

    public void updateOdometry() {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroAngle(), getModulePositions());
        this.field.setRobotPose(getPose());
    }

    public void setWheelsX() {
        setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        });
    }

    public void setWheelsUp() {
        // TODO: get Yaw from gyro method and move to new class
        setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getYaw())),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getYaw())),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getYaw())),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getYaw()))
        });
    }

    public void resetEncoders() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.resetEncoders();
        }
    }

    public void resetOdometry(Pose2d pose) {
        // TODO: get Yaw from gyro method and move to new class
        poseEstimator.resetPosition(
                getYaw(),
                getModulePositions(),
                pose);
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    private Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            positions[modNum] = swerveModules[modNum].getPosition();
        }
        return positions;
    }

    public void getYaw() {
    }
}
