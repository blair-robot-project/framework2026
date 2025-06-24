package frc.team449.subsystems.superstructure.intake

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

object IntakeConstants {
  /*
  * MOVE CORAL RIGHT:
  * left motor pos voltage
  * right motor neg voltage
  * MOVE CORAL LEFT:
  * right motor pos voltage
  * left motor neg voltage
  * INTAKE CORAL:
  * all motors pos
  * OUTTAKE CORAL:
  * all motors neg
  * INTAKE ALGAE:
  * top motor neg
  * OUTTAKE ALGAE:
  * top motor pos
  * HOLD ALGAE:
  * top motor neg
  * */
  const val TOP_MOTOR_ID = 47
  const val RIGHT_MOTOR_ID = 46
  const val LEFT_MOTOR_ID = 48

  val TOP_INVERTED = InvertedValue.Clockwise_Positive
  val RIGHT_INVERTED = InvertedValue.CounterClockwise_Positive
  val LEFT_INVERTED = InvertedValue.Clockwise_Positive
  val BRAKE_MODE = NeutralModeValue.Brake

  const val SUPPLY_LIM = 40.0
  const val STATOR_LIM = 80.0

  // voltage for different scenarios and motors
  const val TOP_CORAL_INWARDS_VOLTAGE = 10.0
  const val TOP_CORAL_OUTTAKE_VOLTAGE = -7.0
  const val TOP_L1_OUTTAKE = -2.1678

  const val SIDES_CORAL_INWARDS_VOLTAGE = 2.0
  const val SIDES_CORAL_OUTTAKE_VOLTAGE = -2.0
  const val SIDES_RUN_TO_SIDE_VOLTAGE = 7.0

  const val CORAL_OUTTAKE_VOLTAGE = -7.0

  const val ALGAE_INTAKE_VOLTAGE = -11.0
  const val ALGAE_HOLD_VOLTAGE = -4.0
  const val ALGAE_OUTTAKE_VOLTAGE = 11.0
  const val ALGAE_STALL_VOLTAGE_THRESHOLD = 30.0

  const val HOLDING_FINISH_VELOCITY = 0.25
  const val CORAL_CENTER_WAIT_TIME = 0.3

  const val RIGHT_CORAL_SENSOR_CAN_ID = 20
  const val MIDDLE_CORAL_SENSOR_CAN_ID = 21
  const val LEFT_CORAL_SENSOR_CAN_ID = 22
  const val BACK_CORAL_SENSOR_CAN_ID = 23

  // Minimum distance in mm on the LaserCAN sensors to count as a detection
  const val CORAL_DETECTION_THRESHOLD = 50
}
