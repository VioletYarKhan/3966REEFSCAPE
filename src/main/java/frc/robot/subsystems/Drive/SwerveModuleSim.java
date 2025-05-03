package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;                      // motion‑profiled PID :contentReference[oaicite:4]{index=4}
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;                       // position struct 
import edu.wpi.first.math.kinematics.SwerveModuleState;                          // state struct 
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Robot;

/**
 * Sim implementation that emulates MAXSwerveModule’s profiled‑PID sim.
 */
public class SwerveModuleSim extends SubsystemBase implements SwerveModuleIO {
  private final ProfiledPIDController simDrivingPID =
      new ProfiledPIDController(
          SimulationConstants.driveP, 
          SimulationConstants.driveI, 
          SimulationConstants.driveD,
          new TrapezoidProfile.Constraints(
             SimulationConstants.maxSpeedMps, 
             SimulationConstants.maxAccelMps2));              // match real robot limits :contentReference[oaicite:7]{index=7}

  private final ProfiledPIDController simTurningPID =
      new ProfiledPIDController(
          SimulationConstants.turnP, 
          SimulationConstants.turnI, 
          SimulationConstants.turnD,
          new TrapezoidProfile.Constraints(
             SimulationConstants.maxAngularSpeed, 
             SimulationConstants.maxAngularAccel));         // full‑rotation constraints :contentReference[oaicite:8]{index=8}

  private double simDrivingSpeed = 0.0;   // current sim speed (m/s)
  private double simTurningPos   = 0.0;   // current sim steer angle (rad)

  private double m_chassisAngularOffset;

  public SwerveModuleSim(double chassisOffset) {
    m_chassisAngularOffset = chassisOffset;
    simTurningPID.enableContinuousInput(-Math.PI, Math.PI);            // wrap at ±π :contentReference[oaicite:9]{index=9}
  }

  /** Returns sim or real state. */
  @Override
  public SwerveModuleState getState() {
    // in sim: use profiled‑PID outputs as the measured state
    double speed = simDrivingSpeed;
    Rotation2d angle = new Rotation2d(simTurningPos - m_chassisAngularOffset);
    return new SwerveModuleState(speed, angle);
  }

  /** Returns sim or real position. */
  @Override
  public SwerveModulePosition getPosition() {
    if (Robot.isReal()) {
      return new SwerveModulePosition(0, new Rotation2d());
    }
    double dist = simDrivingSpeed * 0.02;    // approximate integration (m) per 20 ms step
    return new SwerveModulePosition(dist, new Rotation2d(simTurningPos - m_chassisAngularOffset));
  }

  /** Arms the profiled‑PID loops toward the desired speed & angle. */
  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // apply chassis offset
    SwerveModuleState corrected = new SwerveModuleState(
        desiredState.speedMetersPerSecond,
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)));
    // optimize steer motion 
    corrected.optimize(new Rotation2d(simTurningPos));

    // configure profiled‑PIDs’ goals
    simDrivingPID.setGoal(corrected.speedMetersPerSecond);
    simTurningPID.setGoal(corrected.angle.getRadians());
  }

  /** Zero sim encoders. */
  @Override
  public void resetEncoders() {
    simDrivingSpeed = 0.0;
    simTurningPos   = m_chassisAngularOffset;
    simDrivingPID.reset(simDrivingSpeed);    // reset internal integrators & profiles
    simTurningPID.reset(simTurningPos);
  }

  /** Advance both profiled‑PIDs by 20 ms, updating simDrivingSpeed & simTurningPos. */
  @Override
  public void periodic() {
    // next speed (m/s)
    simDrivingSpeed = simDrivingPID.calculate(simDrivingSpeed);
    // next steer angle (rad)
    simTurningPos   = simTurningPID.calculate(simTurningPos);

    // (optionally clamp to physical maxs)
    simDrivingSpeed = MathUtil.clamp(simDrivingSpeed, -SimulationConstants.maxSpeedMps, SimulationConstants.maxSpeedMps);
  }
}
