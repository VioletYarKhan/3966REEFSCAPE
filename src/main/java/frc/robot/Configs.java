package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(60);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.05, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class Elevator {
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

        static {
                elevatorConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80)
                        .inverted(true)
                        .openLoopRampRate(0)
                        .closedLoopRampRate(0);
                elevatorConfig.encoder
                    .positionConversionFactor(1)
                    .velocityConversionFactor(1);
                elevatorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                   .pid(0.1, 0, 0.01)
                    .velocityFF(0)
                    .outputRange(-0.2, 0.5); // down speed -0.35 max
                        

                elevatorFollowerConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(80);
                elevatorFollowerConfig
                        .follow(10, true);

        }
    }

    public static final class Wrist {
        public static final SparkFlexConfig wristConfig = new SparkFlexConfig();

        static {
                wristConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80)
                        .inverted(false)
                        .openLoopRampRate(0)
                        .closedLoopRampRate(0);
                wristConfig.encoder
                        .positionConversionFactor(1)
                        .velocityConversionFactor(1);
                wristConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.1, 0, 0)
                        .velocityFF(0)
                        .outputRange(-0.35, 0.4);
                }
        }

        public static final class Funnel {
                public static final SparkMaxConfig funnelConfig = new SparkMaxConfig();

                static {
                        funnelConfig
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(80)
                                .inverted(false)
                                .openLoopRampRate(0)
                                .closedLoopRampRate(0);
                        funnelConfig.encoder
                                .positionConversionFactor(1)
                                .velocityConversionFactor(1);
                        funnelConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(0.05, 0, 0)
                                .velocityFF(0)
                                .outputRange(-0.2, 0.2);
                }
        }
}
