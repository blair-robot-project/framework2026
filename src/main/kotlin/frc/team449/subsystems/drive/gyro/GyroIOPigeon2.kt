package frc.team449.subsystems.drive.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import frc.team449.subsystems.drive.SwerveConstants

class GyroIOPigeon2 : GyroIO {
  private val pigeon = Pigeon2(SwerveConstants.PIGEON_CAN_ID)
  private val yaw: StatusSignal<Angle> = pigeon.yaw
  private val yawVelocity: StatusSignal<AngularVelocity> = pigeon.angularVelocityZWorld

  init {
    pigeon.configurator.apply(Pigeon2Configuration())

    // resetting gyro
    pigeon.configurator.setYaw(0.0)
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, yaw, yawVelocity)
    pigeon.optimizeBusUtilization()
  }

  /** Updates the set of loggable inputs. */
  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity) == StatusCode.OK
    // pigeon2 is ccw+
    inputs.yawPosition = pigeon.rotation2d
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.valueAsDouble)
  }
}
