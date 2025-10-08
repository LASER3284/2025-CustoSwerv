
package org.laser3284.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.laser3284.Constants;

import com.ctre.phoenix6.hardware.CANcoder;

/**
 * @class {@link Module}
 * @brief This class is meant to act as a subsystem in Command based framework,
 * which allows swerve module functionality.
 *
 * In order to use this as a command-based subsystem, you should create
 * instances of it, and feed them to the command scheduler, or to the {@link
 * Drive} constructor and consumed.
 *
 * To make things consistent, an azimuth angle of 0 degrees should be
 * perpendicular to the plane of the robot's front face. Our heading should be
 * from -180 to 180 degrees. Our drive velocity is positive if it is heading in
 * the direction of the azimuth, negative otherwise.
 *
 * For SysID, see
 * https://frcdocs.wpi.edu/en/latest/docs/software/advanced-controls/system-identification/
 */
public class Module extends SubsystemBase {
    private final SparkMax azimuthMotor;
    private final SparkFlex driveMotor;
    private final CANcoder azimuthEncodor;
    private final PIDController drivePid;
    private final SimpleMotorFeedforward driveFf;
    private final PIDController azimuthPid;
    private final String name;
    private final Command defaultCommand;

    private SwerveModuleState goal;
    private boolean forceAngle = false;

    private SysIdRoutine driveSysId;
    private SysIdRoutine azimuthSysId;

    /**
     * @brief Creates a new module from a provided set of CAN IDs and PID
     * controllers.
     * @param driveId CAN ID of the {@link SparkFlex} (REV Neo Vortex) drive
     * motor.
     * @param azimuthId CAN ID of the {@link SparkMax} (REV Neo) azimuth motor.
     * @param encoderId CAN ID of the {@link CANcoder} for azimuth absolute
     * positioning.
     * @param name The {@link String} key for NT4.
     * @param drivePid The drive motor PID controller for this module.
     * @param driveFf The feedforward controller for the drive motor for this
     * module.
     * @param azimuthPid The azimuth motor PID controller constants for this module.
     * @see {@link Constants.PidConstants}
     * @see {@link Constants.FeedForwardConstants}
     */
    public Module(int driveId, int azimuthId, int encoderId, String name,
            Constants.PidConstants drivePid,
            Constants.FeedForwardConstants driveFf,
            Constants.PidConstants azimuthPid) {
        // this block will setup the necessary objects of our instance.
        {
            this.driveMotor = new SparkFlex(driveId, MotorType.kBrushless);
            this.azimuthMotor = new SparkMax(azimuthId, MotorType.kBrushless);
            this.azimuthEncodor = new CANcoder(encoderId);

            if (Constants.IS_COMPETITION) {
                // we're gonna ignore any errors from this :P
                this.azimuthEncodor.optimizeBusUtilization();
            }

            this.drivePid = new PIDController(drivePid.p, drivePid.i, drivePid.d);

            this.driveFf = new SimpleMotorFeedforward(
                    driveFf.s, driveFf.v, driveFf.a
                    );

            this.azimuthPid = new PIDController(
                    azimuthPid.p, azimuthPid.i, azimuthPid.d
                    );

            // create a default goal state
            this.goal = new SwerveModuleState(Units.FeetPerSecond.of(0), new Rotation2d(Units.Radians.of(0)));

            this.name = name;

            // NOTE: we use degrees on this PID controller!
            this.azimuthPid.enableContinuousInput(-180, 180);
        }

        // This block will set the inversion of the motors. if you want to do
        // this via REV hardware client, then comment this out.
        {
            SparkBaseConfig driveConfig = new SparkFlexConfig()
                .inverted(Constants.DriveTrainConstants.General.DRIVE_INVERT);

            this.driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            SparkBaseConfig aziConfig = new SparkMaxConfig()
                .inverted(Constants.DriveTrainConstants.General.AZIMUTH_INVERT);

            this.azimuthMotor.configure(aziConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        this.defaultCommand = this.runEnd(() -> this.enabled(), () -> {
            this.driveMotor.setVoltage(0);
            this.azimuthMotor.setVoltage(0);
            this.drivePid.reset();
            this.azimuthPid.reset();
        }).ignoringDisable(false);
        CommandScheduler.getInstance().setDefaultCommand(this, this.defaultCommand);

        // here we're going to set up SysID

        this.driveSysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                // 1 V/s
                Units.Volts.of(1).div(Units.Seconds.of(1)),
                // 8V - slightly better since swerve requires more power due to
                // higher inertia on this mechanism than most others
                Units.Volts.of(8),
                // 10s timeout
                null
                ),
            new SysIdRoutine.Mechanism(
                this::setDriveVoltsWithHeading,
                (SysIdRoutineLog l) -> {
                    // for the drive motor, we want linear units, such as
                    // meters, since we need to control how far and how fast the
                    // motor is moving.
                    l.motor(name)
                        .voltage(this.getDriveVolts())
                        .linearPosition(this.getDrivePosition())
                        .linearVelocity(this.getSpeed());
                },
                this,
                this.name
                )
            );

        this.azimuthSysId = new SysIdRoutine(
            // we're gonna use the default b/c this has less inertia; static
            // friction is already accounted for by SysId
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setAzimuthVoltsWithDrive,
                (SysIdRoutineLog l) -> {
                    // for the azimuth, we need angular units, so that's what we
                    // use here.
                    l.motor(name)
                        .voltage(this.getAzimuthVolts())
                        .angularPosition(this.getAngle())
                        .angularVelocity(this.getAzimuthVelocity());
                },
                this,
                this.name
                )
            );
    }

    @Override
    public void periodic() {
        // we don't want to spam stuff during comp, especially when it's not
        // being used...
        if (!Constants.IS_COMPETITION) {
            SmartDashboard.putNumber(this.name + "/goal/angleDegrees", this.goal.angle.getDegrees());
            SmartDashboard.putNumber(this.name + "/goal/speedMps", this.goal.speedMetersPerSecond);
            SmartDashboard.putNumber(this.name + "/state/driveMps", this.getSpeed().abs(Units.MetersPerSecond));
            SmartDashboard.putNumber(this.name + "/state/driveFps", this.getSpeed().abs(Units.FeetPerSecond));
            SmartDashboard.putNumber(this.name + "/state/aziDegrees", this.getAngle().abs(Units.Degrees));
        }

        this.goal.optimize(new Rotation2d(this.getAngle()));
    }

    /**
     * @brief This is a method that should be called during a scheduling loop of
     * a command. We don't do it in `periodic` since that would possibly incur a
     * performance penalty, and it's better to rely on this for running while
     * the robot is enabled.
     *
     * @implNote I don't think this is very "idiomatic" Java, since we're having
     * to create quite a few new objects for each time this method is called,
     * and that's kinda bad, especially since there's not a direct way to clone
     * a SwerveModuleState, but I'm not immediately aware of any better way to
     * do this.
     */
    public void enabled() {
        var currentAngle = new Rotation2d(this.getAngle());
        var goal = new SwerveModuleState(this.goal.speedMetersPerSecond, this.goal.angle);
        goal.cosineScale(currentAngle);

        // realistically, our motor controllers cannot physically apply more
        // than our battery's voltage, but theoretically this should never
        // really be above about 12.5, maybe 13 if you know your batteries are
        // good.
        var driveVoltage = Units.Volts.of(drivePid.calculate(
            this.getSpeed().abs(Units.MetersPerSecond),
            goal.speedMetersPerSecond
        ) + driveFf.calculate(goal.speedMetersPerSecond));

        // our azimuth doesn't need a feedforward since it won't experience as
        // much inertial resistance.
        var aziVoltage = Units.Volts.of(azimuthPid.calculate(
            this.getAngle().abs(Units.Degrees),
            goal.angle.getDegrees()
        ));

        // if our goal state has a speed >1% of our maximum wheel speed or we're
        // forcing the current angle of the module, then we should apply our
        // calculated voltage. else, ignore azimuth.
        //
        // this is an optimization to prevent unnecessary voltage spinning on
        // the azimuth, which can't get as precise when the drive motor isn't
        // moving fast enough.
        if (Math.abs(goal.speedMetersPerSecond) > Constants.DriveTrainConstants.General.MAX_WHEEL_VELOCITY.abs(Units.MetersPerSecond) * 0.01
                || this.forceAngle) {
            azimuthMotor.setVoltage(aziVoltage);
        } else {
            azimuthMotor.setVoltage(0);
        }

        driveMotor.setVoltage(driveVoltage);
    }

    /**
     * @brief Obtains the linear speed of the drive wheel, relative to the wheel
     * heading.
     * @return {@link LinearVelocity} which represents the surface speed our
     * wheel.
     */
    public LinearVelocity getSpeed() {
        // our motor should be spinning faster than our wheel, so we divide the
        // gear ratio out, which gives us the wheel RPM
        var currentRpm = this.driveMotor.getEncoder().getVelocity()
            / Constants.DriveTrainConstants.General.DRIVE_GEARING;

        // Units are as follows, although some of the typing is lost since we
        // don't specify that wheel circumference is actually inches/turn.
        //
        // 1. (inches / turn) / (seconds / turn)
        // 2. (inches / turn) * (turn / second)
        // 3. inches / second
        //
        // To get seconds per turn, we divide 60 seconds per minute by our
        // current turns per minute, in a similar fashion as above.
        var currentFps = Constants.DriveTrainConstants.General.WHEEL_CIRCUMFERENCE
            .div(Units.Seconds.of(60 / currentRpm));

        return currentFps;
    }

    /**
     * @brief Obtains the current absolute heading angle of the swerve module,
     * relative to the front facing vector of our robot.
     * @return {@link Angle} which represents the wheel heading.
     *
     * @implNote This method will always fetch from the CANcoder, which can
     * create extra CAN bus noise since we're generally accessing the azimuth
     * motor also. This can be alleviated by using the azimuth motor relative
     * encoder most of the time and only using the CANcoder every so often.
     */
    public Angle getAngle() {
        return this.azimuthEncodor.getPosition().getValue();
    }

    /**
     * @brief This is only useful if the goal is currently being used, such as
     * when the robot is enabled. It will set the goal that the robot should aim
     * for.
     * @param goal The swerve module state that we want to achieve, which is
     * optimized by our periodic method.
     */
    public void setGoal(SwerveModuleState goal) {
        this.setGoal(goal, false);
    }

    /**
     * @brief This is only useful if the goal is currently being used, such as
     * when the robot is enabled. It will set the goal that the robot should aim
     * for.
     * @param goal The swerve module state that we want to achieve, which is
     * optimized by our periodic method.
     * @param forceAngle If true, then we will force the module to always
     * attempt to reach the rotational goal, otherwise we'll say "good enough"
     * within a certain margin.
     */
    public void setGoal(SwerveModuleState goal, boolean forceAngle) {
        this.goal = goal;
        this.forceAngle = forceAngle;
    }

    /**
     * @brief This will return the default command created when the object was
     * instantiated.
     * @return A reference to the default {@link Command}, which can be
     * cancelled if need be.
     */
    public Command getDefaultCommand() {
        return this.defaultCommand;
    }

    /**
     * @brief Obtains the current state of the swerve module (drive speed and
     * azimuth angle).
     * @return A new SwerveModuleState object containing the above mentioned
     * paramters.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getSpeed(), new Rotation2d(this.getAngle()));
    }

    /**
     * Fetches the encoder distance within the drive motor's encoder. This value
     * is relative to power on.
     *
     * @return Linear {@link Distance} measurement of how far the wheel has
     * traveled.
     */
    public Distance getDrivePosition() {
        var currentRotations = this.driveMotor.getEncoder().getPosition()
            / Constants.DriveTrainConstants.General.DRIVE_GEARING;

        return Constants.DriveTrainConstants.General.WHEEL_CIRCUMFERENCE
            .times(currentRotations);
    }

    /**
     * Fetches the current position of the swerve module as a
     * {@link SwerveModulePosition}.
     *
     * @return The current position of the drive motor and the heading of the
     * azimuth.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.getDrivePosition(), new Rotation2d(this.getAngle()));
    }

    /**
     * @brief Sets the voltage on the drive motor. This can be useful for custom
     * swerve functionality that's not easily implemented across a single
     * instance (e.g SysID).
     * @param volts {@link Voltage} to send to the drive motor.
     */
    protected void setDriveVolts(Voltage volts) {
        this.driveMotor.setVoltage(volts);
    }

    /**
     * @brief Sets the voltage on the drive motor. This can be useful for custom
     * swerve functionality that's not easily implemented across a single
     * instance (e.g SysID).
     * @param volts Voltage to send to the drive motor.
     */
    protected void setDriveVolts(double volts) {
        this.driveMotor.setVoltage(volts);
    }

    /**
     * Fetches the voltage currently being applied to the motor. Used mostly
     * just for SysID.
     *
     * @return A {@link Voltage} nominally -12 to 12 Volts.
     */
    public Voltage getDriveVolts() {
        // motor.get is all we have access to, but we know it's a percentage of
        // voltage in either + or - direction, so we can just times 12.0 to get
        // the volts and then use units to convert type
        return Units.Volts.of(this.driveMotor.get() * 12.0);
    }

    /**
     * Fetches the voltage currently being applied to the motor. Used mostly
     * just for SysID.
     *
     * @return A {@link Voltage} nominally -12 to 12 Volts.
     */
    public Voltage getAzimuthVolts() {
        // motor.get is all we have access to, but we know it's a percentage of
        // voltage in either + or - direction, so we can just times 12.0 to get
        // the volts and then use units to convert type
        return Units.Volts.of(this.azimuthMotor.get() * 12.0);
    }

    /**
     * Set the azimuth motor to a given voltage.
     *
     * @param volts Voltage to apply to the azimuth.
     */
    protected void setAzimuthVolts(Voltage volts) {
        this.azimuthMotor.setVoltage(volts);
    }

    /**
     * Fetch the angular velocity of the azimuth of this module. Used mostly
     * just for SysID.
     *
     * @return {@link AngularVelocity} from the encoder representing how quickly
     * the azimuth is rotating.
     */
    public AngularVelocity getAzimuthVelocity() {
        return this.azimuthEncodor.getVelocity().getValue();
    }

    // BEGIN SYSID SECTION //

    /**
     * Drive the wheel with a given voltage, while keeping the azimuth heading
     * stable.
     *
     * @param volts The voltage to apply to the drive motor.
     */
    protected void setDriveVoltsWithHeading(Voltage volts) {
        // our azimuth should be locked at 0 degrees.
        var aziVolts = this.azimuthPid.calculate(
                this.getAngle().abs(Units.Degrees),
                0
                );

        this.azimuthMotor.setVoltage(aziVolts);
        this.driveMotor.setVoltage(volts);
    }

    /**
     * Sets the voltage of the azimuth motor to a given voltage, and the drive
     * motor to 0V.
     *
     * @param volts The voltage (nominally -12 to 12 Volts) to set the azimuth
     * to.
     */
    protected void setAzimuthVoltsWithDrive(Voltage volts) {
        this.setDriveVolts(0);
        this.setAzimuthVolts(volts);
    }

    /**
     * Gets the command for running the quasistatic SysID routine for drive.
     * This will lock the azimuth to a set goal heading (0 degrees) and then run
     * SysID on the drive motor.
     *
     * @param dir The direction in which to run the test
     * @return Corresponding {@link Command} that depends on this {@link Module}
     */
    public Command driveSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return this.driveSysId.quasistatic(dir);
    }

    /**
     * Gets the command for running the dynamic SysID routine for drive. This
     * will lock the azimuth to a set goal heading (0 degrees) and then run
     * SysID on the drive motor.
     *
     * @param dir The direction in which to run the test
     * @return Corresponding {@link Command} that depends on this {@link Module}
     */
    public Command driveSysIdDynamic(SysIdRoutine.Direction dir) {
        return this.driveSysId.dynamic(dir);
    }

    /**
     * Gets the command for running the quasistatic SysID routine for azimuth.
     * This will lock the drive motor to 0V and run SysID on the azimuth motor.
     *
     * @param dir The direction in which to run the test.
     * @return Corresponding {@link Command} that depends on this {@link Module}
     */
    public Command azimuthSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return this.azimuthSysId.quasistatic(dir);
    }

    /**
     * Gets the command for running the dynamic SysID routine for azimuth. This
     * will lock the drive motor to 0V and run SysID on the azimuth motor.
     *
     * @param dir The direction in which to run the test.
     * @return Corresponding {@link Command} that depends on this {@link Module}
     */
    public Command azimuthSysIdDynamic(SysIdRoutine.Direction dir) {
        return this.azimuthSysId.dynamic(dir);
    }

    // END SYSID SECTION //
}

