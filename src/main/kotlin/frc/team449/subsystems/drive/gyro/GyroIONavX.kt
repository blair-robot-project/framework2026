package frc.team449.subsystems.drive.gyro

import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units

public class GyroIONavX : GyroIO {
    private val navX: AHRS = AHRS(AHRS.NavXComType.kMXP_SPI)

    /** Updates the set of loggable inputs. */
    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = navX.isConnected
        inputs.yawPosition = Rotation2d.fromDegrees(-navX.angle)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians((-navX.rawGyroZ).toDouble())
    }
}
