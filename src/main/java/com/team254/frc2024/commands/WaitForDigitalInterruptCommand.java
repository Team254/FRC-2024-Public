package com.team254.frc2024.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command for tripping banner sensors
 */
public class WaitForDigitalInterruptCommand extends Command {
    public enum Edges {
        RISING,
        FALLING,
        BOTH
    }

    private List<Consumer<Edges>> usafeActions = new ArrayList<>();
    private AtomicBoolean tripped = new AtomicBoolean(false);
    private AsynchronousInterrupt interrupt;

    public WaitForDigitalInterruptCommand(DigitalInput dio, Edges triggerMode) {
        setName("WaitForDigitalInterruptCommand");
        interrupt = new AsynchronousInterrupt(dio, (triggeredByRisingEdge, triggeredByFallingEdge) -> {
            System.out.println("Got tripped: " + triggeredByRisingEdge + ", " + triggeredByFallingEdge);
            Edges trigger = triggeredByRisingEdge ? Edges.RISING : Edges.FALLING;

            if (triggerMode == Edges.BOTH || trigger == triggerMode) {
                synchronized (this) {
                    this.usafeActions.forEach(action -> action.accept(trigger));
                }
                this.tripped.set(true);
            }
        });
        interrupt.setInterruptEdges(triggerMode == Edges.BOTH || triggerMode == Edges.RISING,
                triggerMode == Edges.BOTH || triggerMode == Edges.FALLING);
    }

    @Override
    public void initialize() {
        tripped.set(false);
        interrupt.enable();
    }

    @Override
    public boolean isFinished() {
        return tripped.get();
    }

    @Override
    public void end(boolean interrupted) {
        interrupt.disable();
    }

    /**
     * Adds an immediate (not thread safe, not scheduled) execution to the
     * interrupt. Use this with caution.
     */
    public synchronized WaitForDigitalInterruptCommand addUnsafeImmediateAction(Consumer<Edges> triggerAction) {
        usafeActions.add(triggerAction);
        return this;
    }

}
