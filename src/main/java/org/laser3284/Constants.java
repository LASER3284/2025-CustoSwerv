// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.laser3284;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * @brief Global constants which are used throughout the code.
 */
public final class Constants {
    public static class PidConstants {
        public final double p, i, d;

        PidConstants(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
        }
    }

    public static class FeedForwardConstants {
        public final double s, v, a;

        FeedForwardConstants(double s, double v, double a) {
            this.s = s;
            this.v = v;
            this.a = a;
        }

        FeedForwardConstants(double s, double v) {
            this.s = s;
            this.v = v;
            this.a = 0;
        }
    }

    public static class OperatorConstants {
        public static final int DRIVE_PORT = 0;
    }

    /**
     * @brief Certain optimizations are only actually useful during the
     * competition, so we should just ignore them when we're not at comp.
     * @warning MAKE SURE THIS IS SET CORRECTLY!
     */
    public static final boolean IS_COMPETITION = false;

    // in the 
    public static class DriveTrainConstants {
        // ignore the warnings (probably); they seem to be an issue with my
        // JDTLS setup? gradle does not show any such warnings. also I can't
        // read them bc of an nvim weirdness...
        //
        // PID units are a lot more abstract than FF (SVA) units, but honestly
        // this is fine since the PID controller (probably) only accepts
        // `double` anyway
        public static class FrontLeft {
            public static final Translation2d FRAME_LOCATION = new Translation2d(
                    1,
                    1
                );

            public static final PidConstants DRIVE_PID = new PidConstants(0, 0, 0);
            public static final PidConstants AZIMUTH_PID = new PidConstants(0, 0, 0);
            public static final FeedForwardConstants DRIVE_FF = new FeedForwardConstants(0, 0);

            // TODO: CHANGE THIS
            public static final int DRIVE_ID = -1;
            public static final int AZIMUTH_ID = -1;
            public static final int ENCODER_ID = -1;
        }

        public static class FrontRight {
            public static final Translation2d FRAME_LOCATION = new Translation2d(
                    1,
                    -1
                );

            public static final PidConstants DRIVE_PID = new PidConstants(0, 0, 0);
            public static final PidConstants AZIMUTH_PID = new PidConstants(0, 0, 0);
            public static final FeedForwardConstants DRIVE_FF = new FeedForwardConstants(0, 0);

            // TODO: CHANGE THIS
            public static final int DRIVE_ID = -1;
            public static final int AZIMUTH_ID = -1;
            public static final int ENCODER_ID = -1;
        }

        public static class BackLeft {
            public static final Translation2d FRAME_LOCATION = new Translation2d(
                    -1,
                    1
                );

            public static final PidConstants DRIVE_PID = new PidConstants(0, 0, 0);
            public static final PidConstants AZIMUTH_PID = new PidConstants(0, 0, 0);
            public static final FeedForwardConstants DRIVE_FF = new FeedForwardConstants(0, 0);

            // TODO: CHANGE THIS
            public static final int DRIVE_ID = -1;
            public static final int AZIMUTH_ID = -1;
            public static final int ENCODER_ID = -1;
        }

        public static class BackRight {
            public static final Translation2d FRAME_LOCATION = new Translation2d(
                    -1,
                    -1
                );

            public static final PidConstants DRIVE_PID = new PidConstants(0, 0, 0);
            public static final PidConstants AZIMUTH_PID = new PidConstants(0, 0, 0);
            public static final FeedForwardConstants DRIVE_FF = new FeedForwardConstants(0, 0);

            // TODO: CHANGE THIS
            public static final int DRIVE_ID = -1;
            public static final int AZIMUTH_ID = -1;
            public static final int ENCODER_ID = -1;
        }

        public static class General {
            /**
             * @brief Whether to tell the motor controller on the drive motor to
             * invert its voltage output.
             *
             * This is in {@link General} b/c we make the assumption every
             * module is the same model and has the same motors.
             */
            public static final boolean DRIVE_INVERT = false;
            /**
             * @brief Whether to tell the motor controller on the azimuth motor
             * to invert its voltage output.
             *
             * This is in {@link General} b/c we make the assumption every
             * module is the same model and has the same motors.
             */
            public static final boolean AZIMUTH_INVERT = false;

            public static final double DRIVE_GEARING = 8.14;
            public static final Distance WHEEL_CIRCUMFERENCE = Units.Inches.of(4.0 * Math.PI);

            /**
             * @brief This value is specified for SDS Mk4 L1 swerve modules with
             * REV Neo Vortex motors. This is free speed, however, and is never
             * achievable on a real robot.
             */
            public static final LinearVelocity MAX_WHEEL_VELOCITY = Units.FeetPerSecond.of(14.5);

            public static final PidConstants DRIVE_PID = new PidConstants(0, 0, 0);
            public static final PidConstants AZIMUTH_PID = new PidConstants(0, 0, 0);
            public static final FeedForwardConstants DRIVE_FF = new FeedForwardConstants(0, 0);
        }
    }
}
