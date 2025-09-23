package org.firstinspires.ftc.teamcode.Commands;

public class InstantCommand extends FunctionalCommand{

    public InstantCommand(Runnable toRun, Subsystem... requirements) {
        super(toRun, ()->{}, (interrupted)->{}, ()->true, requirements);
    }

    public InstantCommand() {
        this(()->{});
    }

}
