package com.hamosad1657.lib

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/*
* Credit to team 6328 Mechanical Advantage.
*
* Here is a link to their GitHub repository:
* https://github.com/Mechanical-Advantage/NetworkAlerts
*
* Here is a link to their explanation on Chief Delphi:
* https://www.chiefdelphi.com/t/systems-checks-tips-tricks-stories/440369/3
*/

/** Class for managing persistent alerts to be sent over NetworkTables. */
class Alert {
	var text: String

	private val type: AlertType
	private var active = false
	private var activeStartTime = 0.0

	/**
	 * Creates a new Alert in the specified group.
	 *
	 * If this is the first to be instantiated in its group,
	 * the appropriate entries will be added to NetworkTables.
	 *
	 * @param group Group identifier, also used as NetworkTables title
	 * @param text Text to be displayed when the alert is active.
	 * @param type Alert level specifying urgency.
	 */
	constructor(group: String, text: String, type: AlertType) {
		this.text = text
		this.type = type

		groups
			.getOrPut(group) {
				SendableAlerts().also { SmartDashboard.putData(group, it) }
			}
			.alerts.add(this)
	}

	/**
	 * Creates a new Alert in the default group - "Alerts".
	 *
	 * If this is the first to be instantiated, the appropriate entries will be added to NetworkTables.
	 *
	 * @param text Text to be displayed when the alert is active.
	 * @param type Alert level specifying urgency.
	 */
	constructor(text: String, type: AlertType) : this("Alerts", text, type)

	/**
	 * Sets whether the alert should currently be displayed.
	 *
	 * When activated, the alert text will also be sent to the console.
	 */
	fun set(active: Boolean) {
		if (active && !this.active) {
			activeStartTime = Timer.getFPGATimestamp()
			when (type) {
				AlertType.ERROR -> DriverStation.reportError(text, false)
				AlertType.WARNING -> DriverStation.reportWarning(text, false)
				AlertType.INFO -> println(text)
			}
		}
		this.active = active
	}

	private class SendableAlerts : Sendable {
		val alerts = mutableListOf<Alert>()

		fun getStrings(type: AlertType): Array<String> = alerts
			.filter { it.type == type && it.active }
			.sortedWith { a1, a2 -> (a2.activeStartTime - a1.activeStartTime).toInt() }
			.map { it.text }
			.toTypedArray()

		override fun initSendable(builder: SendableBuilder) {
			builder.setSmartDashboardType("Alerts")
			builder.addStringArrayProperty("Errors", { getStrings(AlertType.ERROR) }, null)
			builder.addStringArrayProperty("Warnings", { getStrings(AlertType.WARNING) }, null)
			builder.addStringArrayProperty("Info", { getStrings(AlertType.INFO) }, null)
		}
	}

	/**
	 * Represents an alert's level of urgency.
	 */
	enum class AlertType {
		/**
		 * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
		 * for problems which will seriously affect the robot's functionality and thus require immediate
		 * attention.
		 */
		ERROR,

		/**
		 * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
		 * type for problems which could affect the robot's functionality but do not necessarily require
		 * immediate attention.
		 */
		WARNING,

		/**
		 * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type
		 * for problems which are unlikely to affect the robot's functionality, or any other alerts
		 * which do not fall under "ERROR" or "WARNING".
		 */
		INFO,
	}

	companion object {
		private val groups: MutableMap<String, SendableAlerts> = HashMap()
	}
}
