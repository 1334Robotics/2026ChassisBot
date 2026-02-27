package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj.GenericHID;

public class Input {
    // Keyboard0 on robot port 0: WASD keys + E/R rotation
    //   axis 0 = A (dec) / D (inc)  — strafe left/right
    //   axis 1 = W (dec) / S (inc)  — forward/back
    //   axis 2 = E (dec) / R (inc)  — rotation (slow ramp, keyRate 0.01)
    // Keyboard2 on robot port 2: Arrow keys + Q/F buttons
    //   axis 0 = Left (dec) / Right (inc) — strafe left/right
    //   axis 1 = Up (dec) / Down (inc)    — forward/back
    //   button 5 = Q (rotate left), button 6 = F (rotate right)
    private static final GenericHID kb0 = new GenericHID(0);
    private static final GenericHID kb2 = new GenericHID(2);

    /**
     * Translation X (strafe left/right).
     * Returns the larger-magnitude value between WASD (A/D) and arrow keys.
     */
    public static double getTranslationX() {
        double wasd   = kb0.getRawAxis(0);
        double arrows = kb2.getRawAxis(0);
        return (Math.abs(arrows) > Math.abs(wasd)) ? arrows : wasd;
    }

    /**
     * Translation Y (forward/back).
     * Returns the larger-magnitude value between WASD (W/S) and arrow keys.
     */
    public static double getTranslationY() {
        double wasd   = kb0.getRawAxis(1);
        double arrows = kb2.getRawAxis(1);
        return (Math.abs(arrows) > Math.abs(wasd)) ? arrows : wasd;
    }

    /**
     * Rotation.
     * Sources (picks whichever has the largest magnitude):
     * - Keyboard0 axis 2: E (dec = left) / R (inc = right) — smooth ramp via keyRate
     * - Keyboard2 button 5 (Q) = rotate left (-1.0), button 6 (F) = rotate right (+1.0)
     */
    public static double getRotation() {
        // E/R keys on Keyboard0 (axis 2) — smooth ramp rotation
        double erRot = kb0.getRawAxis(2);

        // Q/F keys on Keyboard2 (buttons) — instant full rotation
        double qfRot = 0;
        if (kb2.getRawButton(5)) qfRot -= 1.0;  // Q = rotate left (CCW)
        if (kb2.getRawButton(6)) qfRot += 1.0;  // F = rotate right (CW)

        return (Math.abs(qfRot) > Math.abs(erRot)) ? qfRot : erRot;
    }

    public static void testInput() {
        System.out.printf("X: %.2f  Y: %.2f  Rot: %.2f  (kb0: %.2f/%.2f/%.2f  kb2: %.2f/%.2f  Q:%b F:%b)%n",
            getTranslationX(), getTranslationY(), getRotation(),
            kb0.getRawAxis(0), kb0.getRawAxis(1), kb0.getRawAxis(2),
            kb2.getRawAxis(0), kb2.getRawAxis(1),
            kb2.getRawButton(5), kb2.getRawButton(6));
    }

    /** Get the Keyboard2 HID for button binding */
    public static GenericHID getKeyboard() {
        return kb2;
    }

    /** Create a CommandGenericHID wrapper for Keyboard2 */
    public static CommandGenericHID getCommandKeyboard() {
        return new CommandGenericHID(2);
    }
}
