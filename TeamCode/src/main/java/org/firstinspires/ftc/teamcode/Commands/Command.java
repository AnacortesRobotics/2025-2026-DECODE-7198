package org.firstinspires.ftc.teamcode.Commands;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public abstract class Command {

    private final Set<Subsystem> requirements = new HashSet<>();

    private boolean interruptable = false;

    private String name = "unnamed";

    public void init() {}

    public void run() {}

    public void stop(boolean interrupted) {}

    public boolean isFinished() {return false;}

    public Command addRequirements(Subsystem... subsystems) {
        requirements.addAll(Arrays.asList(subsystems));
        return this;
    }

    public Command addRequirements(Collection<Subsystem> subsystems) {
        requirements.addAll(subsystems);
        return this;
    }

    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    public SequentialCommandGroup andThen(Command command) {
        return new SequentialCommandGroup(this, command);
    }

    public SequentialCommandGroup andThen(Runnable toRun, Subsystem... requirements) {
        return andThen(new InstantCommand(toRun, requirements));
    }

    public Command setInterruptable(boolean interruptable) {
        this.interruptable = interruptable;
        return this;
    }

    public boolean isInterruptable() {
        return interruptable;
    }

    public Command setName(String name) {
        this.name = name;
        return this;
    }

    public String getName() {
        return name;
    }

    public Command cancel() {
        return new InstantCommand(()->{CommandScheduler.getInstance().cancelCommand(this);});
    }

}
