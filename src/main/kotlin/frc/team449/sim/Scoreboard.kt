package frc.team449.sim

import edu.wpi.first.wpilibj.Alert
import org.ironmaple.simulation.SimulatedArena

// Keeps track of points scored during simulation
object Scoreboard {
  var isBlue: Boolean = false

  private var redScore: Alert = Alert("Scoreboard", "Red Score: ?", Alert.AlertType.kWarning)
  private var blueScore: Alert = Alert("Scoreboard", "Blue Score: ?", Alert.AlertType.kWarning)
  private var alerts: Array<Alert> = arrayOf()

  fun score(points: Int, type: String) {
    SimulatedArena.getInstance().addToScore(isBlue, points)
    alerts += Alert("Scoreboard", "${if (isBlue) "Blue" else "Red"}  Scored: $points ($type)", Alert.AlertType.kInfo)
  }

  fun display() {
    blueScore.text = "Blue Score: ${SimulatedArena.getInstance().getScore(true)}"
    blueScore.set(true)
    redScore.text = "Red Score: ${SimulatedArena.getInstance().getScore(false)}"
    redScore.set(true)
    alerts.forEach { it.set(true) }
  }
}
