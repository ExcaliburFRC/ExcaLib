package frc.excalib.additional_utilities;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static frc.excalib.additional_utilities.Color.Colors.*;

/**
 * The LEDs class is responsible for controlling an addressable LED strip.
 * It provides various patterns and commands to manipulate the LED strip's colors and animations.
 */
public class LEDs extends SubsystemBase {
    private final AddressableLED LedStrip = new AddressableLED(LEDS_PORT); // LED strip object
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH); // Buffer to hold LED colors

    private static LEDs instance = null; // Singleton instance of the LEDs class
    private final Random rnd = new Random(); // Random generator for random patterns

    private int tailIndex = 0; // Index for train patterns
    private double offset = 0; // Offset for animations

    // PWM port for the LED strip
    public static final int LEDS_PORT = 9;
    // The length of the LED strip
    public static final int LENGTH = 50;

    // Predefined color arrays
    Color[] orange = new Color[LENGTH];
    Color[] black = new Color[LENGTH];

    /**
     * Private constructor to initialize the LED strip and set the default command.
     */
    private LEDs() {
        LedStrip.setLength(LENGTH);
        LedStrip.start();

        Arrays.fill(orange, ORANGE.color);
        Arrays.fill(black, OFF.color);

        setDefaultCommand(setPattern(LEDPattern.EXPAND, BLUE.color, WHITE.color));
    }

    /**
     * Returns the singleton instance of the LEDs class.
     *
     * @return The LEDs instance.
     */
    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }

    /**
     * Creates a command to set a specific LED pattern with main and accent colors.
     *
     * @param pattern     The LED pattern to set.
     * @param mainColor   The main color for the pattern.
     * @param accentColor The accent color for the pattern.
     * @return A command to execute the pattern.
     */
    public Command setPattern(LEDPattern pattern, Color mainColor, Color accentColor) {
        Command command = new InstantCommand(); // Default command
        Color[] colors = new Color[LENGTH]; // Array to hold LED colors
        int trainLength = (int) (LENGTH / 5.0); // Length of the train pattern
        final AtomicBoolean invert = new AtomicBoolean(false); // Direction for train patterns

        switch (pattern) {
            case OFF:
                // Turns off all LEDs
                Arrays.fill(colors, OFF.color);
                command = new RunCommand(() -> setLedStrip(colors), this).withName("OFF");
                break;

            case SOLID:
                // Sets all LEDs to the main color
                Arrays.fill(colors, mainColor);
                command = new RunCommand(() -> setLedStrip(colors), this).withName("SOLID: " + mainColor.toString());
                break;

            case ALTERNATING_STATIC:
                // Creates a static alternating pattern of main and accent colors
                for (int i = 0; i < LENGTH; i++) {
                    colors[i] = mainColor;
                    colors[i + 1] = accentColor;
                    i++;
                }
                command = new RunCommand(() -> setLedStrip(colors), this)
                        .withName("ALTERNATING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case ALTERNATING_MOVING:
                // Creates a moving alternating pattern of main and accent colors
                AtomicReference<Color> mainAlternatingColor = new AtomicReference<>(mainColor);
                AtomicReference<Color> accentAlternatingColor = new AtomicReference<>(accentColor);
                AtomicReference<Color> tempAlternatingColor = new AtomicReference<>(mainColor);

                command = this.runOnce(() -> {
                            for (int i = 0; i < LENGTH - 1; i++) {
                                colors[i] = mainAlternatingColor.get();
                                colors[i + 1] = accentAlternatingColor.get();
                                i++;
                            }
                            setLedStrip(colors);

                            tempAlternatingColor.set(mainAlternatingColor.get());
                            mainAlternatingColor.set(accentAlternatingColor.get());
                            accentAlternatingColor.set(tempAlternatingColor.get());
                        })
                        .andThen(new WaitCommand(0.25)).repeatedly()
                        .withName("ALTERNATING_MOVING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case RANDOM:
                // Sets LEDs to random colors
                command = this.runOnce(() -> {
                            Arrays.fill(colors, new Color(rnd.nextInt(255), rnd.nextInt(255), rnd.nextInt(255)));
                            setLedStrip(colors);
                        }).andThen(new WaitCommand(1))
                        .repeatedly().withName("RANDOM");
                break;

            case BLINKING:
                // Creates a blinking pattern between main and accent colors
                command = Commands.repeatingSequence(
                        new InstantCommand(() -> {
                            Arrays.fill(colors, mainColor);
                            setLedStrip(colors);
                        }, this),
                        new WaitCommand(0.15),
                        new InstantCommand(() -> {
                            Arrays.fill(colors, accentColor);
                            setLedStrip(colors);
                        }, this),
                        new WaitCommand(0.15)
                ).withName("BLINKING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case EXPAND:
                // Creates an expanding pattern from the center
                final AtomicInteger expandStep = new AtomicInteger(0);
                command = new InstantCommand(() -> {
                    Color[] LedColors = new Color[LENGTH];
                    Arrays.fill(LedColors, mainColor);
                    int LEDS_LENGTH = 5;

                    int step = expandStep.get();

                    if (LENGTH % 2 == 0) {
                        int leftCenter = (LENGTH / 2) - 1;
                        int rightCenter = LENGTH / 2;
                        int leftIndex = leftCenter - step;
                        int rightIndex = rightCenter + step;

                        for (int i = 0; i < LEDS_LENGTH; i++) { // Light up 3 LEDs per side
                            if (leftIndex - i >= 0) {
                                LedColors[leftIndex - i] = accentColor;
                            }
                            if (rightIndex + i < LENGTH) {
                                LedColors[rightIndex + i] = accentColor;
                            }
                        }
                    } else {
                        int center = LENGTH / 2;
                        int leftIndex = center - step;
                        int rightIndex = center + step;

                        for (int i = 0; i < LEDS_LENGTH; i++) { // Light up 3 LEDs per side
                            if (leftIndex - i >= 0) {
                                LedColors[leftIndex - i] = accentColor;
                            }
                            if (rightIndex + i < LENGTH) {
                                LedColors[rightIndex + i] = accentColor;
                            }
                        }
                    }

                    int newStep = expandStep.get() + 1;
                    if (newStep > (LENGTH / 2)) {
                        newStep = 0;
                    }
                    expandStep.set(newStep);

                    setLedStrip(LedColors);
                }, this)
                        .andThen(new WaitCommand(0.01))
                        .repeatedly()
                        .withName("EXPAND, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            // Additional cases omitted for brevity
            default:
                break;
        }

        return command.ignoringDisable(true);
    }

    public Command setPattern(LEDPattern pattern, Color color) {
        return setPattern(pattern, color, OFF.color);
    }


    public Command setLEDsCommand(Color[] colors) {
        return this.runOnce(() -> setLedStrip(colors)).ignoringDisable(true);
    }

    public Command twoStatesCommand(LEDPattern firstPattern, Color firstColor, LEDPattern secondPattern, Color secondColor, Trigger switchStates) {
        return setPattern(firstPattern, firstColor).until(switchStates).andThen(setPattern(secondPattern, secondColor));
    }

    public Command deadLineLEDcommand(Command ledCommand) {
        return new StartEndCommand(
                ledCommand::schedule,
                restoreLEDs()::schedule);
    }

    public Command scheduleLEDcommand(Command ledCommand) {
        return new InstantCommand(ledCommand::schedule);
    }

    public Command circleLEDs(Color[] colors) {
        return Commands.repeatingSequence(
                        setLEDsCommand(colors),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> shiftLeds(colors)))
                .ignoringDisable(true);
    }

    public Command restoreLEDs() {
        return new InstantCommand(() ->
                CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(this)));
    }

    public enum LEDPattern {
        OFF,
        RAINBOW,
        SOLID,
        ALTERNATING_STATIC,
        ALTERNATING_MOVING,
        TRAIN_CIRCLE,
        TRAIN,
        RANDOM,
        EXPAND,
        BLINKING,
        RSL;
    }

    public Color getAllianceColor() {
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) return Color.Colors.TEAM_BLUE.color;
        else if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) return Color.Colors.RED.color;
        else return Color.Colors.WHITE.color;
    }

    private void setLedStrip(Color[] colors) {
        for (int i = 0; i < colors.length; i++) buffer.setLED(i, colors[i]);
        LedStrip.setData(buffer);
    }

    private void shiftTrain(Color[] colors, Color mainColor, Color trainColor, int trainLength, int offset) {
        tailIndex = findTailIndex(colors, trainColor);
        Arrays.fill(colors, mainColor);
        for (int i = 0; i < trainLength; i++) {
            colors[stayInBounds(i + tailIndex + offset, colors.length)] = trainColor;
        }
    }

    private int findHeadIndex(Color[] colors, Color trainColor) {
        for (int i = colors.length - 1; i >= 0; i--) {
            if (colors[i].equals(trainColor)) return i;
        }
        return -1;
    }

    private int findTailIndex(Color[] colors, Color trainColor) {
        for (int i = 0; i < colors.length; i++) {
            if (colors[i].equals(trainColor)) return i;
        }
        return -1;
    }

    private void shiftLeds(Color[] colors) {
        Color lastColor = colors[colors.length - 1];

        for (int i = colors.length - 2; i >= 0; i--) {
            colors[i + 1] = colors[i];
        }

        colors[0] = lastColor;
    }

    private int stayInBounds(int value, int length) {
        if (value >= length) return stayInBounds(value - length, length);
        if (value < 0) return stayInBounds(value + length, length);
        return value;
    }
}