package frc.team449

import edu.wpi.first.math.geometry.Pose3d

object RobotVisual {

  private val components = Array(5) { Pose3d() }

  fun getComponents(): Array<Pose3d> = components

  fun update() {
    // TODO: Update Robot Visual
  }
}
