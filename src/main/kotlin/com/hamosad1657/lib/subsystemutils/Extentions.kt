package com.hamosad1657.lib.subsystemutils

import edu.wpi.first.wpilibj2.command.SubsystemBase

/** Sets the name of the subsystem to the name of the class. */
fun SubsystemBase.setNameToClassName() {
    this.name = this.javaClass.kotlin.simpleName
}