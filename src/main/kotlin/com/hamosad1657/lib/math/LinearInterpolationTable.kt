package frc.robot.Utilities

import java.awt.geom.Point2D

/* This should be updated to use binary search at some point */
class LinearInterpolationTable(vararg points: Point2D) {
	private var m_maxInput = Double.NEGATIVE_INFINITY
	private var m_minInput = Double.POSITIVE_INFINITY
	val table: Array<Point2D>
	val size: Int

	init {
		table = points as Array<Point2D>
		size = table.size
		for (i in 0 until size) {
			if (table[i].x > m_maxInput) {
				m_maxInput = table[i].x
			}
			if (table[i].x < m_minInput) {
				m_minInput = table[i].x
			}
		}
	}

	fun getOutput(input: Double): Double {
		var index = 0
		if (input <= m_minInput) {
			index = 0
		} else if (input >= m_maxInput) {
			index = size - 2
		} else {
			for (i in 1 until table.size) {
				if (input > table[i - 1].x && input <= table[i].x) {
					index = i - 1
				}
			}
		}
		return interpolate(input, table[index], table[index + 1])
	}

	val x: DoubleArray
		get() {
			val xVals = DoubleArray(size)
			for (i in 0 until size) {
				xVals[i] = table[i].x
			}
			return xVals
		}
	val y: DoubleArray
		get() {
			val yVals = DoubleArray(size)
			for (i in 0 until size) {
				yVals[i] = table[i].y
			}
			return yVals
		}

	companion object {
		fun interpolate(input: Double, point1: Point2D, point2: Point2D): Double {
			val slope = (point2.y - point1.y) / (point2.x - point1.x)
			val delta_x = input - point1.x
			val delta_y = delta_x * slope
			return point1.y + delta_y
		}
	}
}