

package org.laser3284.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.laser3284.Constants;
import org.laser3284.Constants.DriveTrainConstants;

/**
 * @class Drive
 * @brief A basic implementation of a swerve drivetrain. Many of the members are
 * protected rather than private so that this class can be extended as a parent,
 * which would give it additional functionality (such as better pose
 * estimation or SysID).
 * @warning This drivetrain does not implement any functionality for Autonomous
 * control. That can be done via rewrite or via subclassing.
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

    protected Command defaultCommand;

    public Drive() {
        this.defaultCommand = this.runEnd(() -> this.enabled(), () -> {});

        CommandScheduler.getInstance().registerSubsystem(this);
        CommandScheduler.getInstance().setDefaultCommand(this, this.defaultCommand);
    }

    @Override
    public void periodic() {
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
        // TODO
    }

    public void setJoystickSuppliers(DoubleSupplier leftX, DoubleSupplier leftY,
            DoubleSupplier rightX, DoubleSupplier rightY) {
        this.leftXJoystick = leftX;
        this.leftYJoystick = leftY;
        this.rightXJoystick = rightX;
        this.rightYJoystick = rightY;
    }

    public void setPerspective(Perspective view) {
        this.currentView = view;
    }

    public void enabled() {
        // TODO
    }
}
