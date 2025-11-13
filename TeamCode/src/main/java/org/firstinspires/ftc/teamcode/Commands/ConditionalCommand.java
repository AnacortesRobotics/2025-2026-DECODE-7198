package org.firstinspires.ftc.teamcode.Commands;

import java.util.function.BooleanSupplier;

public class ConditionalCommand extends FunctionalCommand {

    public ConditionalCommand(Runnable toRun, BooleanSupplier condition, Subsystem... requirements) {
        super(()->{}, condition.getAsBoolean() ? toRun : ()->{}, (interrupted)->{}, condition, requirements);
    }

    public ConditionalCommand() {
        this(()->{}, ()->true);
    }

}
