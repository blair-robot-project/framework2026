package frc.team449.sim

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Degrees
import org.ironmaple.simulation.SimulatedArena

class BunnybotsArena() : SimulatedArena(BunnybotsFieldMap()) {

  class BunnybotsFieldMap : FieldMap() {

    init {
      // Coordinates are relative to bottom left of field, with blue on the left

      // AdvantageScope Model Offset
      val offset = Translation2d(0.673100/2+0.088900*1.5, -0.431800/2+0.088900/2)

      // Bottom Wall
      super.addBorderLine(
        Translation2d(0.088900, 0.088900).plus(offset),
        Translation2d(16.548100, 0.088900).plus(offset)
      )
      // Left Wall
      super.addBorderLine(
        Translation2d(0.088900, 0.088900).plus(offset),
        Translation2d(0.088900, 8.318500).plus(offset)
      )
      // Top Wall
      super.addBorderLine(
        Translation2d(0.088900, 8.318500).plus(offset),
        Translation2d(16.548100, 8.318500).plus(offset)
      )
      // Right Wall
      super.addBorderLine(
        Translation2d(16.548100, 8.318500).plus(offset),
        Translation2d(16.548100, 0.088900).plus(offset)
      )

      // Blue Barrier
      super.addBorderLine(
        Translation2d(0.088900, 5.575300).plus(offset),
        Translation2d(2.330450, 5.575300).plus(offset)
      )
      super.addBorderLine(
        Translation2d(2.330450, 5.575300).plus(offset),
        Translation2d(2.330450, 2.819400).plus(offset)
      )
      super.addBorderLine(
        Translation2d(2.330450, 2.819400).plus(offset),
        Translation2d(2.940050, 2.819400).plus(offset)
      )
      super.addBorderLine(
        Translation2d(2.940050, 2.819400).plus(offset),
        Translation2d(2.940050, 6.184900).plus(offset)
      )
      super.addBorderLine(
        Translation2d(2.940050, 6.184900).plus(offset),
        Translation2d(0.088900, 6.184900).plus(offset)
      )

      // Red Border
      super.addBorderLine(
        Translation2d(16.548100, 5.575300).plus(offset),
        Translation2d(14.306550, 5.575300).plus(offset)
      )
      super.addBorderLine(
        Translation2d(14.306550, 5.575300).plus(offset),
        Translation2d(14.306550, 2.819400).plus(offset)
      )
      super.addBorderLine(
        Translation2d(14.306550, 2.819400).plus(offset),
        Translation2d(13.696950, 2.819400).plus(offset)
      )
      super.addBorderLine(
        Translation2d(13.696950, 2.819400).plus(offset),
        Translation2d(13.696950, 6.184900).plus(offset)
      )
      super.addBorderLine(
        Translation2d(13.696950, 6.184900).plus(offset),
        Translation2d(16.548100, 6.184900).plus(offset)
      )

      // Star Spires

      // Left
      super.addBorderLine(
        Translation2d(0.088900, 7.251700).plus(offset),
        Translation2d(0.190500, 7.251700).plus(offset)
      )
      super.addBorderLine(
        Translation2d(0.190500, 7.251700).plus(offset),
        Translation2d(0.190500, 6.642100).plus(offset)
      )
      super.addBorderLine(
        Translation2d(0.190500, 6.642100).plus(offset),
        Translation2d(0.088900, 6.642100).plus(offset)
      )

      // Top Left
      super.addBorderLine(
        Translation2d(1.612900, 8.318500).plus(offset),
        Translation2d(1.612900, 8.216900).plus(offset)
      )
      super.addBorderLine(
        Translation2d(1.612900, 8.216900).plus(offset),
        Translation2d(2.222500, 8.216900).plus(offset)
      )
      super.addBorderLine(
        Translation2d(2.222500, 8.216900).plus(offset),
        Translation2d(2.222500, 8.318500).plus(offset)
      )

      // Top Right
      super.addBorderLine(
        Translation2d(14.414500, 8.318500).plus(offset),
        Translation2d(14.414500, 8.216900).plus(offset)
      )
      super.addBorderLine(
        Translation2d(14.414500, 8.216900).plus(offset),
        Translation2d(15.024100, 8.216900).plus(offset)
      )
      super.addBorderLine(
        Translation2d(15.024100, 8.216900).plus(offset),
        Translation2d(15.024100, 8.318500).plus(offset)
      )

      // Right
      super.addBorderLine(
        Translation2d(16.548100, 7.251700).plus(offset),
        Translation2d(16.446500, 7.251700).plus(offset)
      )
      super.addBorderLine(
        Translation2d(16.446500, 7.251700).plus(offset),
        Translation2d(16.446500, 6.642100).plus(offset)
      )
      super.addBorderLine(
        Translation2d(16.446500, 6.642100).plus(offset),
        Translation2d(16.548100, 6.642100).plus(offset)
      )

      // Cosmic Converters

      // Bottom Left
      super.addBorderLine(
        Translation2d(0.088900, 1.130300).plus(offset),
        Translation2d(0.952500, 1.130300).plus(offset)
      )
      super.addBorderLine(
        Translation2d(0.952500, 1.130300).plus(offset),
        Translation2d(0.952500, 0.088900).plus(offset)
      )

      // Bottom Right
      super.addBorderLine(
        Translation2d(16.548100, 1.130300).plus(offset),
        Translation2d(15.684500, 1.130300).plus(offset)
      )
      super.addBorderLine(
        Translation2d(15.684500, 1.130300).plus(offset),
        Translation2d(15.684500, 0.088900).plus(offset)
      )

      // Top Left
      super.addBorderLine(
        Translation2d(0.088900, 4.549775).plus(offset),
        Translation2d(0.952500, 4.549775).plus(offset)
      )
      super.addBorderLine(
        Translation2d(0.952500, 4.549775).plus(offset),
        Translation2d(0.952500, 5.591175).plus(offset)
      )

      // Top Right
      super.addBorderLine(
        Translation2d(16.548100, 4.530725).plus(offset),
        Translation2d(15.684500, 4.530725).plus(offset)
      )
      super.addBorderLine(
        Translation2d(15.684500, 4.530725).plus(offset),
        Translation2d(15.684500, 5.572125).plus(offset)
      )

      // Field Obstacles

      // Center Box [Bottom Left Corner: (7.883525, 3.684588), Top Right Corner: (8.753475, 4.722813), Center: (8.3185, 4.2037005) ]
      super.addRectangularObstacle(0.869950, 1.038225, Pose2d(
        Translation2d(8.318500, 4.203700).plus(offset),
        Rotation2d()
      ) )

      // Top Left Box
      super.addRectangularObstacle(0.609600, 0.869950, Pose2d(
        Translation2d(6.474907, 6.169515).plus(offset),
        Rotation2d(Degrees.of(44.4855955))  // This is the actual rotation from the CAD
      ) )
      // Top Right Box
      super.addRectangularObstacle(0.609600, 0.869950, Pose2d(
        Translation2d(6.474907, 6.169515).plus(offset),
        Rotation2d(Degrees.of(-44.4855955))
      ) )
      // Bottom Left Box
      super.addRectangularObstacle(0.869950, 0.609600,Pose2d(
        Translation2d(6.474907, 2.237885).plus(offset),
        Rotation2d(Degrees.of(-44.4855955))
      ) )
      // Bottom Right Box
      super.addRectangularObstacle(0.869950, 0.609600, Pose2d(
        Translation2d(10.162093, 2.237885).plus(offset),
        Rotation2d(Degrees.of(44.4855955))
      ) )

    }

  }

  override fun placeGamePiecesOnField() {
    super.addGamePiece(Lunite.getLunite(Pose2d(2.0,2.0, Rotation2d())))
  }

  init {
      placeGamePiecesOnField()
  }

}