package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Input {
    public static final XboxController driveController = new XboxController(0);

    public static double getTranslationX() {
        return driveController.getLeftX();
    }

    public static void testInput() {
        // Example usage of getTranslationX and getTranslationY to remove the unused method warning
        double translationX = getTranslationX();
        double translationY = getTranslationY();
        System.out.println("Translation X: " + translationX);
        System.out.println("Translation Y: " + translationY);
    }

    public static double getTranslationY() {
        return driveController.getLeftY();
    }

    public static double getRotation() {
        return driveController.getRightY();
    }
    
}