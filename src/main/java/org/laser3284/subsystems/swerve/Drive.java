
package org.laser3284.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.laser3284.Constants;
import org.laser3284.Constants.DriveTrainConstants;

import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * @class Drive
 * @brief A basic implementation of a swerve drivetrain. Many of the members are
 * protected rather than private so that this class can be extended as a parent,
 * which would give it additional functionality (such as better pose
 * estimation or SysID).
 * @warning This drivetrain does not implement any functionality for Autonomous
 * control. That can be done via rewrite or via subclassing.
 * @implNote we're using a continuous interval for rotation of [-180,180]; this
 * is done by 
 * 
 * TODO: finish
 */
public class Drive extends SubsystemBase {
    public final String NAME = "drivetrain";
    protected final Module frontRight = new Module(
            DriveTrainConstants.FrontRight.DRIVE_ID,
            DriveTrainConstants.FrontRight.DRIVE_ID,
            DriveTrainConstants.FrontRight.ENCODER_ID,
            NAME + "/frontright",
            Constants.DriveTrainConstants.FrontRight.DRIVE_PID,
            Constants.DriveTrainConstants.FrontRight.DRIVE_FF,
            Constants.DriveTrainConstants.FrontRight.AZIMUTH_PID
            );

    protected final Module frontLeft = new Module(
            DriveTrainConstants.FrontLeft.DRIVE_ID,
            DriveTrainConstants.FrontLeft.DRIVE_ID,
            DriveTrainConstants.FrontLeft.ENCODER_ID,
            NAME + "/frontleft",
            Constants.DriveTrainConstants.FrontLeft.DRIVE_PID,
            Constants.DriveTrainConstants.FrontLeft.DRIVE_FF,
            Constants.DriveTrainConstants.FrontLeft.AZIMUTH_PID
            );

    protected final Module backRight = new Module(
            DriveTrainConstants.BackRight.DRIVE_ID,
            DriveTrainConstants.BackRight.DRIVE_ID,
            DriveTrainConstants.BackRight.ENCODER_ID,
            NAME + "/backright",
            Constants.DriveTrainConstants.BackRight.DRIVE_PID,
            Constants.DriveTrainConstants.BackRight.DRIVE_FF,
            Constants.DriveTrainConstants.BackRight.AZIMUTH_PID
            );

    protected final Module backLeft = new Module(
            DriveTrainConstants.BackLeft.DRIVE_ID,
            DriveTrainConstants.BackLeft.DRIVE_ID,
            DriveTrainConstants.BackLeft.ENCODER_ID,
            NAME + "/backleft",
            Constants.DriveTrainConstants.BackLeft.DRIVE_PID,
            Constants.DriveTrainConstants.BackLeft.DRIVE_FF,
            Constants.DriveTrainConstants.BackLeft.AZIMUTH_PID
            );

    protected DoubleSupplier leftXJoystick;
    protected DoubleSupplier leftYJoystick;
    protected DoubleSupplier rightXJoystick;
    protected DoubleSupplier rightYJoystick;
    protected DoubleSupplier slowFactor;

    protected Command defaultCommand;

    protected Pigeon2 gyro = new Pigeon2(Constants.DriveTrainConstants.GYRO_ID);

    protected SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            Constants.DriveTrainConstants.FrontLeft.FRAME_LOCATION,
            Constants.DriveTrainConstants.FrontRight.FRAME_LOCATION,
            Constants.DriveTrainConstants.BackLeft.FRAME_LOCATION,
            Constants.DriveTrainConstants.BackRight.FRAME_LOCATION
            );

    protected SwerveDrivePoseEstimator poseEstimator;

    protected SwerveModulePosition[] currentPositions;

    public Drive() {
        this.defaultCommand = this.runEnd(() -> this.enabled(), () -> {});

        CommandScheduler.getInstance().registerSubsystem(this);
        CommandScheduler.getInstance().setDefaultCommand(
                this, this.defaultCommand
                );

        this.currentPositions = new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
                this.frontRight.getPosition(),
                this.backLeft.getPosition(),
                this.backRight.getPosition()
        };

        // initialize our pose estimator to zero
        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.swerveKinematics,
                this.gyro.getRotation2d(),
                this.currentPositions,
                Pose2d.kZero
                );
    }

    /**
     * @brief updates the internal state for the swerve module positions. Must
     * be called before updateOdometry.
     */
    protected void updateSwervePositions() {
        this.currentPositions[0] = this.frontLeft.getPosition();
        this.currentPositions[1] = this.frontRight.getPosition();
        this.currentPositions[2] = this.backLeft.getPosition();
        this.currentPositions[3] = this.backRight.getPosition();
    }

    @Override
    public void periodic() {
        // the command scheduler API guarantees all subsystems' `periodic` is
        // called on a command loop, i.e every 20ms. We therefore can assume
        // that our pose estimator is being called every ~20ms, which is the
        // default.
        this.updateSwervePositions();
        this.updateOdometry();
    }

    /**
     * @brief The basis of control directions for the drivetrain.
     */
    public enum Perspective {
        /**
         * @brief Operator (aka Field) perspective; pushing forward on the
         * joystick should move the robot farther from the near driverstation
         * wall.
         */
        Operater,

        /**
         * @brief Robot perspective; pushing forward on the joystick should move
         * the robot further in the direction of its front face.
         */
        Robot
    }
    protected Perspective currentView = Perspective.Operater;

    /**
     * @brief Updates the internal state of the pose estimator and tracks how
     * each of the swerve modules have moved. Can and should be overriden when
     * implmenting a new drivetrain.
     */
    public void updateOdometry() {
        this.poseEstimator.update(
            this.gyro.getRotation2d(),
            this.currentPositions
            );
    }

    public Pose2d getCurrentPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    /**
     * @brief resets the current pose in the pose estimator to use the given
     * pose.
     * @warning This method is dangerous because it can cause erroneous
     * correction of measurements that aren't real.
     */
    public void setCurrentPose(Pose2d pose) {
        this.poseEstimator.resetPose(pose);
    }

    /**
     * @todo document
     */
    public void addVisionMeasurement(Pose2d visionMeasurement,
            double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
        this.poseEstimator.addVisionMeasurement(
                visionMeasurement,
                timestampSeconds,
                visionStdDevs
                );
    }

    /**
     * @todo document
     */
    public void addVisionMeasurement(Pose2d visionMeasurement,
            double timestampSeconds) {
        this.poseEstimator.addVisionMeasurement(
                visionMeasurement,
                timestampSeconds
                );
    }

    /**
     * @brief Allows the drivetrain to be aware of the joystick values for both
     * left and right, on both their x and y.
     */
    public void setJoystickSuppliers(DoubleSupplier leftX, DoubleSupplier leftY,
            DoubleSupplier rightX, DoubleSupplier rightY, DoubleSupplier slowFactor) {
        this.leftXJoystick = leftX;
        this.leftYJoystick = leftY;
        this.rightXJoystick = rightX;
        this.rightYJoystick = rightY;
        this.slowFactor = slowFactor;
    }

    /**
     * @brief set the drivetrain to interpret controller input as either
     * operator perspective (driverstation) or robot perspective.
     * @see Perspective
     */
    public void setPerspective(Perspective view) {
        this.currentView = view;
    }

    public void enabled() {
        // TODO
    }
}
