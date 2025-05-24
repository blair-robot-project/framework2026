package frc.team449.subsystems.superstructure.intake

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Current

object IntakeConstants {
  const val MOTOR_ID = 46

  val CURRENT_LIMIT: Current = Amps.of(30.0)
  const val BRAKE_MODE = true

  const val MOTOR_INVERTED = false

  const val CORAL_INTAKE_VOLTAGE = 11.0
  const val ALGAE_INTAKE_VOLTAGE = 9.0

  const val ALGAE_HOLD_VOLTAGE = 4.15
  const val CORAL_HOLD_VOLTAGE = 0.35

  const val CORAL_OUTTAKE_VOLTAGE = 3.0
  const val CORAL_OUTTAKE_PIVOT_SIDE_VOLTAGE = -3.0
  const val L1_OUTTAKE = -2.1678
  const val ALGAE_OUTTAKE_VOLTAGE = -10.0

  const val DESCORE_ALGAE_VOLTAGE = -3.0

  const val WAIT_AFTER_ALGAE_DETECTED = 0.5

  const val ALGAE_SENSOR_DIO_PORT = 11
  const val PIVOT_SENSOR_DIO_PORT = 12

  const val READY_PIVOT_CORAL_TIME = 0.25

  // TODO Add CAN IDs for new LaserCAN sensors
  const val BOTTOM_CORAL_SENSOR_CAN_ID = 20
  const val LEFT_CORAL_SENSOR_CAN_ID = 21
  const val RIGHT_CORAL_SENSOR_CAN_ID = 22
  const val TOP_CORAL_SENSOR_CAN_ID = 23

  // Minimum distance in mm on the LaserCAN sensors to count as a detection (TODO Calibration)
  const val LASER_CAN_SENSOR_MIN_DISTANCE_MM = 64
}
