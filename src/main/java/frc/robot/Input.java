package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj.GenericHID;

public class Input {
    public static final XboxController driveController = new XboxController(0);
    
    // Keyboard control - port 2 (WPILib simulator keyboard shows up as a joystick)
    private static final GenericHID keyboard = new GenericHID(2);
    
    // Toggle between controller and keyboard
    private static boolean useKeyboard = false;
    
    /**
     * Switch control mode between Xbox controller and keyboard
     */
    public static void toggleControlMode() {
        useKeyboard = !useKeyboard;
        System.out.println("Control mode: " + (useKeyboard ? "KEYBOARD" : "XBOX CONTROLLER"));
    }
    
    /**
     * Set control mode explicitly
     * @param keyboard true for keyboard, false for Xbox controller
     */
    public static void setControlMode(boolean keyboard) {
        useKeyboard = keyboard;
        System.out.println("Control mode set to: " + (useKeyboard ? "KEYBOARD" : "XBOX CONTROLLER"));
    }
    
    /**
     * Check if keyboard mode is active
     */
    public static boolean isKeyboardMode() {
        return useKeyboard;
    }
    
    public static double getTranslationX() {
        if (useKeyboard) {
            return getKeyboardTranslationX();
        }
        return driveController.getLeftX();
    }

    public static void testInput() {
        // Example usage of getTranslationX and getTranslationY to remove the unused method warning
        double translationX = getTranslationX();
        double translationY = getTranslationY();
        System.out.println("Translation X: " + translationX);
        System.out.println("Translation Y: " + translationY);
    }
    
    // getting the information neccessary for driving the robot
    public static double getTranslationY() {
        if (useKeyboard) {
            return getKeyboardTranslationY();
        }
        double value = driveController.getLeftY();
        return value;
    }

    public static double getRotation() {
        if (useKeyboard) {
            return getKeyboardRotation();
        }
        return driveController.getRightX();
    }
    
    // Keyboard controls mapping
    // WASD for translation, Q/E for rotation
    private static double getKeyboardTranslationX() {
        double x = 0;
        if (keyboard.getRawButton(1)) { // A - strafe left
            x -= 0.5;
        }
        if (keyboard.getRawButton(4)) { // D - strafe right
            x += 0.5;
        }
        return x;
    }
    
    private static double getKeyboardTranslationY() {
        double y = 0;
        if (keyboard.getRawButton(3)) { // W - forward
            y += 0.5;
        }
        if (keyboard.getRawButton(2)) { // S - backward
            y -= 0.5;
        }
        return y;
    }
    
    private static double getKeyboardRotation() {
        double rotation = 0;
        if (keyboard.getRawButton(5)) { // Q - rotate left
            rotation -= 0.4;
        }
        if (keyboard.getRawButton(6)) { // E - rotate right
            rotation += 0.4;
        }
        return rotation;
    }
    
    /**
     * Get the keyboard HID for button binding
     */
    public static GenericHID getKeyboard() {
        return keyboard;
    }
    
    /**
     * Create a CommandGenericHID wrapper for the keyboard
     */
    public static CommandGenericHID getCommandKeyboard() {
        return new CommandGenericHID(2);
    }
}