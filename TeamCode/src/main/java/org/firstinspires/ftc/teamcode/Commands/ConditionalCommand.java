package org.firstinspires.ftc.teamcode.Commands;

public class ConditionalCommand extends FunctionalCommand {

    public ConditionalCommand(Runnable toRun, Subsystem... requirements) {
        super(toRun, ()->{}, (interrupted)->{}, ()->true, requirements);
    }

    public ConditionalCommand() {
        this(()->{});
    }

}
