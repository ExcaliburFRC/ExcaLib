package frc.excalib.additional_utilities;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DoubleClickClient {
    public static final double MAX_LENGTH_SECS = 0.4;
    private final Trigger resetTrigger;
    private final Timer resetTimer = new Timer();
    private DoublePressState state = DoublePressState.IDLE;

    private DoubleClickClient(Trigger baseTrigger) {
        resetTrigger = baseTrigger;
    }

    public static Trigger doublePress(Trigger baseTrigger) {
        var tracker = new DoubleClickClient(baseTrigger);
        return new Trigger(tracker::get);
    }

    private boolean get() {
        boolean pressed = resetTrigger.getAsBoolean();
        switch (state) {
            case IDLE:
                if (pressed) {
                    state = DoublePressState.FIRST_PRESS;
                    resetTimer.reset();
                    resetTimer.start();
                }
                break;
            case FIRST_PRESS:
                if (!pressed) {
                    if (resetTimer.hasElapsed(MAX_LENGTH_SECS)) {
                        reset();
                    } else {
                        state = DoublePressState.FIRST_RELEASE;
                    }
                }
                break;
            case FIRST_RELEASE:
                if (pressed) {
                    state = DoublePressState.SECOND_PRESS;
                } else if (resetTimer.hasElapsed(MAX_LENGTH_SECS)) {
                    reset();
                }
                break;
            case SECOND_PRESS:
                if (!pressed) {
                    reset();
                }
        }
        return state == DoublePressState.SECOND_PRESS;
    }

    private void reset() {
        state = DoublePressState.IDLE;
        resetTimer.stop();
    }

    private enum DoublePressState {
        IDLE,
        FIRST_PRESS,
        FIRST_RELEASE,
        SECOND_PRESS
    }
}