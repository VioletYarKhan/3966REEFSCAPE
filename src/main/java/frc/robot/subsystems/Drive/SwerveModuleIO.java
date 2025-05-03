package frc.robot.subsystems.Drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();
    public void setDesiredState(SwerveModuleState desiredState);
    public void resetEncoders();
}
