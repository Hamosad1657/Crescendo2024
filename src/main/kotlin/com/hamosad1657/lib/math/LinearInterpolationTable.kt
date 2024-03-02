package com.hamosad1657.lib.math

typealias Point = Pair<Double, Double>

class LinearInterpolationTable(vararg points: Point) {
	private val table: Array<out Point> = points

	private var minInput = table.minOf { it.first }
	private var maxInput = table.maxOf { it.first }

	fun getOutputFor(input: Double): Double {
		var index = 0

		if (input <= minInput) index = 0
		else if (input >= maxInput) index = table.size - 2
		else {
			for (i in 1 until table.size) {
				if (input > table[i - 1].first && input <= table[i].first) index = i - 1
			}
		}

		return interpolate(input, table[index], table[index + 1])
	}

	val firsts get() = table.map { it.first }.toDoubleArray()
	val seconds get() = table.map { it.second }.toDoubleArray()

	companion object {
		private fun interpolate(input: Double, point1: Point, point2: Point): Double {
			val slope = (point2.second - point1.second) / (point2.first - point1.first)
			val deltaFirst = input - point1.first
			val deltaSecond = deltaFirst * slope
			return point1.second + deltaSecond
		}
	}
}