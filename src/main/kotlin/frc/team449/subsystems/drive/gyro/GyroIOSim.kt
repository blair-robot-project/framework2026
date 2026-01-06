package frc.team449.subsystems.drive.gyro

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.RadiansPerSecond
import frc.team449.util.PhoenixUtil
import org.ironmaple.simulation.drivesims.GyroSimulation

class GyroIOSim(
  val gyroSimulation: GyroSimulation
) : GyroIO {

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.connected = true
    inputs.yawPosition = gyroSimulation.gyroReading
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(
      gyroSimulation.measuredAngularVelocity.`in`(RadiansPerSecond)
    )

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps()
    inputs.odometryYawPositions = gyroSimulation.cachedGyroReadings
  }
}
