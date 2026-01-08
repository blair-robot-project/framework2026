package frc.team449.subsystems.drive.gyro

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

public interface GyroIO {
    @AutoLog
    open class GyroIOInputs {
        @JvmField var connected: Boolean = false

        @JvmField var yawPosition: Rotation2d = Rotation2d()

        @JvmField var yawVelocityRadPerSec: Double = 0.0

        @JvmField var odometryYawTimestamps: DoubleArray = doubleArrayOf()

        @JvmField var odometryYawPositions: Array<Rotation2d> = arrayOf()
    }

    /** Updates the set of loggable inputs. */
    fun updateInputs(inputs: GyroIOInputs) {}
}
