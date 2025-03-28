package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;

public class GamepadNW {
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    Gamepad gamepad;
    Vector<String> clicked = new Vector<>();
    Vector<String> pressed = new Vector<>();
    public GamepadNW (Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public void update() {
        this.previousGamepad.copy(this.currentGamepad);
        this.currentGamepad.copy(this.gamepad);

        List<String> buttons = Arrays.asList(
                "a", "x", "y", "b", "dpad_down", "dpad_left", "dpad_right", "dpad_up",
                "right_bumper", "left_bumper", "back", "start", "right_stick_button", "left_stick_button", "touchpad"
        );

        clicked.clear();
        //pressed.clear();

        for (String button : buttons) {
            try {
                Field field = gamepad.getClass().getField(button);
                boolean currentState = field.getBoolean(currentGamepad);
                boolean previousState = field.getBoolean(previousGamepad);

                if (currentState)
                {
                    if (!pressed.contains(button))
                        pressed.add(button);
                    if (!previousState)
                        clicked.add(button);
                }
                else if (previousState)
                    pressed.remove(button);
            } catch (NoSuchFieldException | IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

    public boolean isClicked(String button){
        if (clicked.contains(button)) {
            //logger.writeLn("clicked " + button);
            return true;
        }
        return false;
    }
    public boolean isPressed(String button){
        return pressed.contains(button);
    }

    public Vector<String> getClicked(){
        return clicked;
    }

    public Vector<String> getPressed(){
        return pressed;
    }
}
