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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a swerve drive style drivetrain.
 */
public class Swerve {
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
                    getYaw(),
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

    public double pitch = getPitch().getDegrees();
    public double roll = getRoll().getDegrees();
    public double yaw = getYaw().getDegrees();


    public Swerve() {
        resetEncoders();
        zeroHeading();
        setBreakMode(true);
        SmartDashboard.putData("Field", field);
    }

    /**
     * Periodic method for the swerve drive.
     */
    public void periodic() {
        updateOdometry();
    }

    /**
     * Set all swerve modules to brake mode or coast mode.
     *
     * @param brakeMode Whether the robot should be in brake mode or coast mode.
     */
    public void setBreakMode(boolean brakeMode) {
        for (MAXSwerveModule swerveModule : swerveModules) {
            // if brakeMode is true, setBrakeMode will be called, otherwise setCoastMode will be called with a one line if statement
            if (brakeMode) { swerveModule.setBrakeMode(); }
            else { swerveModule.setCoastMode(); }
        }
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
                            xSpeedLimiter.calculate(xSpeed) * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                            ySpeedLimiter.calculate(ySpeed) * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                            rotationLimiter.calculate(rotation) * DriveConstants.MAX_ANGULAR_SPEED,
                            getPoseRotation())
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
                            getPoseRotation())
                            : new ChassisSpeeds(
                            xSpeed,
                            ySpeed,
                            rotation));
        }
        setModuleStates(swerveModuleStates);
    }

    /**
     * Updates the field relative position of the robot.
     * This should be called periodically.
     */
    public void updateOdometry() {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
        this.field.setRobotPose(getPose());
    }

    /**
     * Sets the desired state of each module to rotate the wheels in an X pattern.
     */
    public void setWheelsX() {
        setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        });
    }

    /**
     * Sets the desired state of each module to rotate the wheels in the up position.
     */
    public void setWheelsUp() {
        // TODO: get Yaw from gyro method and move to new class
        setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getPoseRotation())),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getPoseRotation())),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getPoseRotation())),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getPoseRotation()))
        });
    }

    /**
     * resets the encoders on all the modules
     */
    public void resetEncoders() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.resetEncoders();
        }
    }

    /**
     * resets the PoseEstimator to the given pose
     *
     * @param pose the pose to reset the odometry to
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                getPoseRotation(),
                getModulePositions(),
                pose);
    }

    /**
     * returns the PoseEstimator to be used in other classes
     *
     * @return poseEstimator is the PoseEstimator object
     */
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * returns the current estimated pose of the robot from the PoseEstimator
     *
     * @return poseEstimator.getEstimatedPosition() is the current estimated pose of the robot
     */
    private Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * sets the desired state of each module to the given states array
     *
     * @param desiredStates the array of desired states for each module
     */
    private void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * returns the current position of each module
     *
     * @return positions is the array of current positions of each module
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            positions[modNum] = swerveModules[modNum].getPosition();
        }
        return positions;
    }

    /**
     * resets the gyro to 0 degrees
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * returns the current estimated heading of the robot in Rotation2d object
     *
     * @return getPose().getRotation() is the current estimated heading of the robot
     */
    public Rotation2d getPoseRotation() {
        return getPose().getRotation();
    }

    /**
     * returns the current heading of the gyro in Rotation object
     *
     * @return yawRotation2d is the current heading of the gyro
     */
    public Rotation2d getYaw() {
        Rotation2d yawRotation2d = Rotation2d.fromDegrees(gyro.getAngle());

        if (DriveConstants.GYRO_REVERSED) {
            yawRotation2d = yawRotation2d.unaryMinus();
        }

        this.yaw = yawRotation2d.getDegrees();

        return yawRotation2d;
    }

    /**
     * returns the current pitch of the gyro in Rotation2d object
     *
     * @return pitchRotation2d is the current pitch of the gyro
     */
    public Rotation2d getPitch() {

        Rotation2d pitchRotation2d = Rotation2d.fromDegrees(gyro.getXComplementaryAngle() - ((gyro.getXComplementaryAngle() > 0) ? 180 : -180));

        this.pitch = pitchRotation2d.unaryMinus().getDegrees();

        return pitchRotation2d;

    }

    /**
     * returns the current roll of the gyro in Rotation2d object
     *
     * @return rollRotation2d is the current roll of the gyro
     */
    public Rotation2d getRoll() {

        Rotation2d rollRotation2d = Rotation2d.fromDegrees(gyro.getYComplementaryAngle() - ((gyro.getYComplementaryAngle() > 0) ? 180 : -180));

        this.roll = rollRotation2d.unaryMinus().getDegrees();

        return rollRotation2d;

    }
}
