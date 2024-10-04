package com.team254.lib.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team254.lib.loops.IStatusSignalLoop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;

// Class which helps manage runnables with conditions to run.
public class ConditionalRunnables implements IStatusSignalLoop {
    public record ConditionalRunnable(BooleanSupplier condition, Runnable runnable) {
    }

    // Map of boolean suppliers to runnable.
    ConcurrentHashMap<String, ConditionalRunnable> runnables = new ConcurrentHashMap<>();

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        // No status signals. Rely on other registered for this.
        return List.of();
    }

    public void register(String key, BooleanSupplier supplier, Runnable runnable) {
        System.out.println("\n\n\nREGISTER: " + key + "\n\n\n\n");
        runnables.put(key, new ConditionalRunnable(supplier, runnable));
        System.out.println("rnsize: " + runnables.size());
    }

    public void remove(String key) {
        System.out.println("\n\n\nDEREGISTER\n\n\n\n");
        runnables.remove(key);
    }

    @Override
    public void onLoop() {
        runnables.forEach(
                (key, cr) -> {
                    System.out.println(key + ", " + cr.condition.getAsBoolean());
                    if (cr.condition.getAsBoolean()) {
                        cr.runnable.run();
                        runnables.remove(key);
                    }
                });
    }

    public static AtomicInteger nonce = new AtomicInteger(0);

    public Command addWaitCommand(BooleanSupplier condition, Runnable action) {
        AtomicBoolean triggered = new AtomicBoolean(false);
        var state = new Object() {
            String key = "";
        };
        return new SequentialCommandGroup(
                Commands.runOnce(() -> {
                    state.key = "k" + nonce.getAndAdd(1);
                    triggered.set(false);
                    this.register(state.key, condition, () -> {
                        triggered.set(true);
                        System.out.println("\n\n\nTRIGGERED\n\n\n\n");
                        action.run();
                    });
                }),
                Commands.waitUntil(triggered::get)).finallyDo(() -> {
                    System.out.println("nstize: " + runnables.size());
                    this.remove(state.key);
                });
    }
}
