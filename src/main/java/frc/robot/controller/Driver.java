package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;

public class Driver {
    XboxController controller;

    public Driver(int port) {
        controller = new XboxController(port);
    }

    public double getvY() {
        return controller.getLeftY();
    }

    public double getvX() {
        return controller.getLeftX();
    }

    public double getRotation() {
        return controller.getRightX();
    }
}