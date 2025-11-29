package frc.team449.util

import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Subsystem

// this class only works iff any and all logic using it are bound to the main scheduler
// because this periodic function in its entirety will have run before any other commands/periodics
// have run, meaning there is no chance for getDeltaTime to be called before the time update is complete!

object Clock : Subsystem {

  private var savedTime = Seconds.mutable(0.0)
  var deltaTime = Seconds.mutable(0.0)

  init {
    savedTime.mut_replace(RobotController.getMeasureFPGATime())
  }

  override fun periodic() {
    val currentTime = RobotController.getMeasureFPGATime()
    deltaTime.mut_replace(currentTime)
    deltaTime.mut_minus(savedTime)
    savedTime.mut_replace(currentTime)
  }

  fun getDeltaTime(): Time? {
    return deltaTime
  }
}
