package com.team254.frc2024.controlboard;

import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ModalControls {
    private static Optional<ModalControls> instance = Optional.empty();

    public enum Mode {
        NOT_SPECIFIED,
        SPEAKER,
        HP,
        AMP,
        POOP,
        CLIMB
    }

    private Mode mode = Mode.NOT_SPECIFIED;
    private Consumer<Mode> stateChangeConsumer;
    private Trigger shootTrigger;
    private Trigger intakeTrigger;

    public static ModalControls getInstance() {
        if (instance.isEmpty()) {
            instance = Optional.of(new ModalControls());
        }
        return instance.get();
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public void registerStateChangeConsumer(Consumer<Mode> consumer) {
        this.stateChangeConsumer = consumer;
    }

    private void maybeTriggerStateChangeConsumer(Mode newMode) {
        if (this.mode != newMode) {
            if (this.stateChangeConsumer != null) {
                this.stateChangeConsumer.accept(newMode);
            }
        }
    }

    public void configureBindings() {
        ControlBoard.getInstance().getAmpMode().and(climbMode().negate()).onTrue(Commands.runOnce(() -> {
            maybeTriggerStateChangeConsumer(Mode.AMP);
            this.mode = Mode.AMP;
        }));

        ControlBoard.getInstance().getHPMode().and(climbMode().negate()).onTrue(Commands.runOnce(() -> {
            maybeTriggerStateChangeConsumer(Mode.HP);
            this.mode = Mode.HP;
        }));

        ControlBoard.getInstance().getAimAtGoalMode().and(climbMode().negate()).onTrue(Commands.runOnce(() -> {
            maybeTriggerStateChangeConsumer(Mode.SPEAKER);
            this.mode = Mode.SPEAKER;
        }));

        ControlBoard.getInstance().getAimAtPoopMode().and(climbMode().negate()).onTrue(Commands.runOnce(() -> {
            maybeTriggerStateChangeConsumer(Mode.POOP);
            this.mode = Mode.POOP;
        }));

        ControlBoard.getInstance().getZeroMode().debounce(0.05).onTrue(Commands.runOnce(() -> {
            maybeTriggerStateChangeConsumer(Mode.NOT_SPECIFIED);
            this.mode = Mode.NOT_SPECIFIED;
        }));

        ControlBoard.getInstance().getClimbMode().debounce(0.05).onTrue(Commands.runOnce(() -> {
            maybeTriggerStateChangeConsumer(Mode.CLIMB);
            this.mode = Mode.CLIMB;
        }));
        shootTrigger = ControlBoard.getInstance().shoot();
        intakeTrigger = ControlBoard.getInstance().intake();
    }

    private Trigger intake() {
        return intakeTrigger
                .and(shootTrigger.negate());
    }

    private Trigger shoot() {
        return shootTrigger.and(intakeTrigger.negate());
    }

    private Trigger intakeAndShoot() {
        return shootTrigger.and(intakeTrigger);
    }

    private Trigger climbArmsUp() {
        return ControlBoard.getInstance().getClimbUp()
                .and(climbMode());
    }

    private Trigger climbArmsDown() {
        return ControlBoard.getInstance().climbArmsDown()
                .and(climbMode());
    }

    private Trigger climbDown() {
        return ControlBoard.getInstance().climbAndScoreTrap()
                .and(climbMode());
    }

    private Trigger climbDutyCycleDown() {
        return ControlBoard.getInstance().climbDutyCycleDown()
                .and(climbMode());
    }

    public Trigger noopMode() {
        return new Trigger(() -> this.mode == Mode.NOT_SPECIFIED);
    }

    public Trigger noopIntake() {
        return intake()
                .and(noopMode());
    }

    public Trigger noopFenderShot() {
        return shoot().and(noopMode());
    }

    public Trigger noopIntakeAndFenderShot() {
        return intakeAndShoot()
                .and(noopMode());
    }

    // Speaker mode
    public Trigger aimAtGoalMode() {
        return new Trigger(() -> this.mode == Mode.SPEAKER);
    }

    public Trigger aimAtGoalShoot() {
        return shoot().and(aimAtGoalMode());
    }

    public Trigger aimAtGoalIntake() {
        return intake()
                .and(aimAtGoalMode());
    }

    public Trigger aimAtGoalIntakeAndShoot() {
        return intakeAndShoot()
                .and(aimAtGoalMode());
    }

    // HP mode
    public Trigger hpMode() {
        return new Trigger(() -> this.mode == Mode.HP);
    }

    public Trigger hpShoot() {
        return shoot()
                .and(hpMode());
    }

    public Trigger hpIntake() {
        return intake()
                .and(hpMode());
    }

    public Trigger hpIntakeAndShoot() {
        return intakeAndShoot()
                .and(hpMode());
    }

    // AMP mode
    public Trigger ampMode() {
        return new Trigger(() -> this.mode == Mode.AMP);
    }

    public Trigger ampShoot() {
        return shoot()
                .and(ampMode());
    }

    public Trigger ampIntake() {
        return intake()
                .and(ampMode());
    }

    // Poop mode
    public Trigger poopMode() {
        return new Trigger(() -> this.mode == Mode.POOP);
    }

    public Trigger poopShoot() {
        return shoot()
                .and(poopMode());
    }

    public Trigger poopIntake() {
        return intake()
                .and(poopMode());
    }

    public Trigger poopIntakeAndShoot() {
        return intakeAndShoot()
                .and(poopMode());
    }

    // Climb Mode
    public Trigger climbMode() {
        return new Trigger(() -> this.mode == Mode.CLIMB);
    }

    public Trigger raiseClimbUp() {
        return climbArmsUp()
                .and(climbMode());
    }

    public Trigger climbAndStageForTrap() {
        return climbDown()
                .and(climbMode());
    }

    public Trigger climbShoot() {
        return shoot()
                .and(climbMode());
    }

    public Trigger moveClimbArmsDown() {
        return climbArmsDown()
                .and(climbMode());
    }

    public Trigger moveClimbDutyCycle() {
        return climbDutyCycleDown()
                .and(climbMode());
    }
}
