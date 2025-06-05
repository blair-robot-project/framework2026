package frc.team449.subsystems.superstructure.climb

import edu.wpi.first.units.Units.Amps

object ClimbConstants {
  const val MOTOR_ID = 49

  val CURRENT_LIMIT = Amps.of(60.0)
  const val BRAKE_MODE = true

  const val INVERTED = false

  const val RUN_VOLTAGE = 12.0
  const val HOLD_VOLTAGE = 4.0

  const val SENSOR_DIO_PORT = 5
}
