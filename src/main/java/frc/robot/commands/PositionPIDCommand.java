package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GryphonLib.MovementCalculations;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PositionPIDCommand extends Command{
    
    public DriveSubsystem drivetrain;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = AutoConstants.kAutoAlignPIDController;

    private final Timer timer = new Timer();

    private final Debouncer endTriggerDebouncer = new Debouncer(0.04);

    private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
    private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();



    private PositionPIDCommand(DriveSubsystem drivetrain, Pose2d goalPose) {
        this.drivetrain = drivetrain;
        this.goalPose = goalPose;
    }

    public static Command generateCommand(DriveSubsystem swerve, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.driveRobotRelativeChassis(new ChassisSpeeds(0,0,0));
            swerve.setX();
        });
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        drivetrain.driveRobotRelativeChassis(
            mDriveController.calculateRobotRelativeSpeeds(
                drivetrain.getCurrentPose(), goalState
            )
        );

        xErrLogger.accept(drivetrain.getCurrentPose().getX() - goalPose.getX());
        yErrLogger.accept(drivetrain.getCurrentPose().getY() - goalPose.getY());
        SmartDashboard.putNumber("PID Control Error", drivetrain.getCurrentPose().relativeTo(goalPose).getTranslation().getNorm());
        SmartDashboard.putNumber("PID Control Velocity", MovementCalculations.getVelocityMagnitude(drivetrain.getCurrentSpeeds()).magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        Pose2d diff = drivetrain.getCurrentPose().relativeTo(goalPose);

        SmartDashboard.putString("PID Align Report", "Adjustments to alginment took: " + timer.get() + " seconds and interrupted = " + interrupted
            + "\nPosition offset: " + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) + " cm"
            + "\nRotation offset: " + diff.getRotation().getMeasure().in(Degrees) + " deg"
            + "\nVelocity value: " + MovementCalculations.getVelocityMagnitude(drivetrain.getCurrentSpeeds()) + "m/s"
        );
    }

    @Override
    public boolean isFinished() {

        Pose2d diff = drivetrain.getCurrentPose().relativeTo(goalPose);

        var rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            Rotation2d.fromDegrees(3.0).getRotations(), 
            0.0, 
            1.0
        );

        var position = diff.getTranslation().getNorm() < Centimeter.of(1.5).in(Meters);

        var speed = MovementCalculations.getVelocityMagnitude(drivetrain.getCurrentSpeeds()).magnitude() < InchesPerSecond.of(2).in(MetersPerSecond);

        // System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);
        
        return endTriggerDebouncer.calculate(
            rotation && position && speed
        );
    }
}