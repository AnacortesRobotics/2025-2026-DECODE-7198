package org.firstinspires.ftc.teamcode.Commands;

public interface Subsystem {

    default String getName() {
        return this.getClass().getSimpleName();
    }

}
