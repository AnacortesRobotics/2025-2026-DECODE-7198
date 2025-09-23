package org.firstinspires.ftc.teamcode.Commands;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public abstract class Command {

    private final Set<Subsystem> requirements = new HashSet<>();

    public void init() {}

    public void run() {}

    public void stop(boolean interrupted) {}

    public boolean isFinished() {return false;}

    public void addRequirements(Subsystem... subsystems) {
        requirements.addAll(Arrays.asList(subsystems));
    }

    public void addRequirements(Collection<Subsystem> subsystems) {
        requirements.addAll(subsystems);
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

}
