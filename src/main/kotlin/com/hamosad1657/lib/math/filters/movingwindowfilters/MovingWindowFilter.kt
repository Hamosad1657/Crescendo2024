package com.hamosad1657.lib.math.filters.movingwindowfilters

import java.util.LinkedList

/** A base class for a moving window filter. */
abstract class MovingWindowFilter {
	/** The number of samples to be included in the calculation. */
	abstract var window: Int

	/** The calculation to use on the samples list in [calculate]. */
	protected abstract val calculation: (LinkedList<Double>) -> Double

	private val latestSamples = LinkedList<Double>()

	/**
	 * @param newSample - The newest sample to include in the calculation.
	 * @return The calculation result, AKA the latest output of the filter.
	 *         If less samples than the window size were provided, then the
	 *         calculation will simply use less samples.
	 */
	fun calculate(newSample: Double): Double {
		while (latestSamples.size >= window) latestSamples.removeLast()
		latestSamples.addFirst(newSample)
		return calculation.invoke(latestSamples)
	}

	/**
	 * Clears all the previous samples and attempts to fill the samples list with new ones.
	 * If less samples than the window size are provided, then the next calculation will
	 * simply use less samples. If more samples than the window size are provided, then
	 * only the first ones will be included.
	 * > To clear all the previous samples without providing new ones, use [clear].
	 */
	fun reset(newValues: DoubleArray) {
		latestSamples.clear()
		if (newValues.isNotEmpty()) {
			for (i in newValues.indices) {
				latestSamples.add(i, newValues[i])
			}
		}
	}

	/**
	 * Clears all the previous samples and attempts to fill the samples list with new ones.
	 * If less samples than the window size are provided, then the next calculation will
	 * simply use less samples. If more samples than the window size are provided, then
	 * only the first ones will be included.
	 * > To clear all the previous samples without providing new ones, use [clear].
	 */
	fun reset(newValues: Collection<Double>) {
		reset(newValues.toDoubleArray())
	}

	fun reset(newValues: Array<Double>) {
		reset(newValues.toDoubleArray())
	}

	/**
	 * Clears all the previous samples, and fills the samples list with the new value.
	 * To clear all the previous samples without providing new ones, use [clear].
	 */
	fun reset(newValue: Double) {
		reset(DoubleArray(window) { newValue })
	}

	/**
	 * Clears all the previous samples.
	 */
	fun clear() {
		latestSamples.clear()
	}
}