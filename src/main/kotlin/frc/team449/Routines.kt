package frc.team449

import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine

open class Routines(
  val autoFactory: AutoFactory
) {

  // do nothing
  fun doNothing(): AutoRoutine {
    val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
    return nothing
  }
}
