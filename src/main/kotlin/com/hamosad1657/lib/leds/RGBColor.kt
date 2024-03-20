package com.hamosad1657.lib.leds

data class RGBColor(val red: Int, val green: Int, val blue: Int) {
	companion object {
		val RED = RGBColor(255, 0, 0)
		val ORANGE = RGBColor(255, 20, 0)
		val YELLOW = RGBColor(255, 140, 0)
		val GREEN = RGBColor(0, 255, 0)
		val CYAN = RGBColor(0, 255, 255)
		val BLUE = RGBColor(0, 0, 255)
		val MAGENTA = RGBColor(255, 0, 255)
		val BLACK = RGBColor(0, 0, 0)
		val WHITE = RGBColor(255, 255, 255)
	}
}