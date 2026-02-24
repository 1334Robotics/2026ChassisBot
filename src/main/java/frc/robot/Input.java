package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj.GenericHID;

public class Input {
    public static final XboxController driveController = new XboxController(0);
    
    // Keyboard0 on robot port 0: WASD = axes 0,1 (translation)
    // Keyboard2 on robot port 2: Arrow keys = axes 0,1 (translation), Q/F = axis 2 (rotation)
    private static final GenericHID kb0 = new GenericHID(0);  // Keyboard0 = WASD
    private static final GenericHID kb2 = new GenericHID(2);  // Keyboard2 = Arrows + Q/F
    
    // Deadband
    private static final double DEADBAND = 0.05;
    
    /**
     * Translation X (strafe):
     *   Keyboard0 axis 0 (A/D keys) OR Keyboard2 axis 0 (Left/Right arrows)
     *   Uses whichever has larger absolute value.
     */
    public static double getTranslationX() {
        double wasd = kb0.getRawAxis(0);
        double arrows = kb2.getRawAxis(0);
        
        if (Math.abs(wasd) < DEADBAND) wasd = 0;
        if (Math.abs(arrows) < DEADBAND) arrows = 0;
        
        return (Math.abs(arrows) > Math.abs(wasd)) ? arrows : wasd;
    }

    /**
     * Translation Y (forward/back):
     *   Keyboard0 axis 1 (W/S keys) OR Keyboard2 axis 1 (Up/Down arrows)
     *   Uses whichever has larger absolute value.
     */
    public static double getTranslationY() {
        double wasd = kb0.getRawAxis(1);
        double arrows = kb2.getRawAxis(1);
        
        if (Math.abs(wasd) < DEADBAND) wasd = 0;
        if (Math.abs(arrows) < DEADBAND) arrows = 0;
        
        return (Math.abs(arrows) > Math.abs(wasd)) ? arrows : wasd;
    }

    /**
     * Rotation:
     *   Keyboard2 button 5 (Q = rotate left), button 6 (F = rotate right)
     *   These are momentary buttons - only active while held down.
     *   Also checks Xbox right stick (kb0 axis 4) if a real controller is connected.
     */
    public static double getRotation() {
        // Xbox right stick (if real controller plugged in)
        double xboxRot = kb0.getRawAxis(4);
        if (Math.abs(xboxRot) < DEADBAND) xboxRot = 0;
        
        // Keyboard rotation via buttons (momentary, no latching)
        double keyRot = 0;
        if (kb2.getRawButton(5)) {  // Q key - rotate left
            keyRot -= 1.0;
        }
        if (kb2.getRawButton(6)) {  // F key - rotate right
            keyRot += 1.0;
        }
        
        return (Math.abs(keyRot) > Math.abs(xboxRot)) ? keyRot : xboxRot;
    }

    public static void testInput() {
        System.out.printf("X: %.2f  Y: %.2f  Rot: %.2f  (kb0: %.2f/%.2f  kb2: %.2f/%.2f/%.2f)%n",
            getTranslationX(), getTranslationY(), getRotation(),
            kb0.getRawAxis(0), kb0.getRawAxis(1),
            kb2.getRawAxis(0), kb2.getRawAxis(1), kb2.getRawAxis(2));
    }
    
    /**
     * Get the keyboard HID for button binding
     */
    public static GenericHID getKeyboard() {
        return kb2;
    }
    
    /**
     * Create a CommandGenericHID wrapper for Keyboard2
     */
    public static CommandGenericHID getCommandKeyboard() {
        return new CommandGenericHID(2);
    }
}