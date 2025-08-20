package frc.team449

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Radians
import kotlin.math.max

object RobotVisual {
  private val pivotBase =
    Pose3d(
      0.0,
      0.0,
      0.0,
      Rotation3d(0.0, 0.0, 0.0),
    )

  private const val FIRST_STAGE_MAX = 0.6
  private const val SECOND_STAGE_MAX = 0.57
  private const val THIRD_STAGE_MAX = 0.56 +0.13

  private val components = Array(5) { Pose3d() }

  fun getComponents(): Array<Pose3d> = components

  fun update(
    pivot: Double,
    elevator: Double,
    wrist: Double,
  ) {
    val totalHeight = elevator.coerceIn(0.0, FIRST_STAGE_MAX + SECOND_STAGE_MAX + THIRD_STAGE_MAX)
    val firstStage = totalHeight.coerceAtMost(FIRST_STAGE_MAX)
    val secondStage = (totalHeight - FIRST_STAGE_MAX).coerceIn(0.0, SECOND_STAGE_MAX)
    val thirdStage = (totalHeight - FIRST_STAGE_MAX - SECOND_STAGE_MAX).coerceIn(0.0, THIRD_STAGE_MAX)

    // pivot
    components[0] =
      pivotBase.transformBy(
        Transform3d(
          Translation3d(
            -0.136,
            0.0,
            0.245,
          ),
          Rotation3d(0.0, -pivot, 0.0),
        ),
      )

    // first stage
    components[1] =
      components[0].transformBy(
        Transform3d(
          Translation3d(firstStage, 0.0, 0.0),
          Rotation3d(),
        ),
      )
// second stage
    components[2] =
      components[0].transformBy(
        Transform3d(
          Translation3d((secondStage + firstStage), 0.0, 0.0),
          Rotation3d(),
        ),
      )
// third stage
    components[3] =
      components[0].transformBy(
        Transform3d(
          Translation3d(thirdStage + secondStage + firstStage, 0.0, 0.0),
          Rotation3d(),
        ),
      )
// wrist
    components[4] =
      components[3].transformBy(
        Transform3d(
          Translation3d(
            //close-ish estimates, might be cooked
            THIRD_STAGE_MAX + ( if (wrist < 1){
              0.02
            } else{
              -0.001
            }),


            0.0,


         if (wrist < 1){
            0.1025
          } else{
            0.071
          }


          ),
          Rotation3d(0.0,-wrist, 0.0),
        ),
      )
  }
}