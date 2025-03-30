package frc.team449.subsystems.superstructure.climb

import edu.wpi.first.units.Units.Amps

object ClimbConstants {
  const val MOTOR_ID = 62

  val CURRENT_LIMIT = Amps.of(30.0)
  const val BRAKE_MODE = true

  const val INVERTED = true

  const val RUN_VOLTAGE = 12.0

  const val SENSOR_DIO_PORT = 1
  const val SENSOR2_DIO_PORT = 5
}
