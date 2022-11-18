package org.firstinspires.ftc.teamcode;

public class ToggleBoolean extends EventListener {
    private boolean value = false;
    private boolean switchVal = false;

    public ToggleBoolean(boolean initalValue) {
        value = initalValue;
    }

    private void updateSwitch() {
        switchVal = !switchVal;
        invoke(switchVal ? "switchTrue" : "switchFalse");
    }
    public void makeTrue() {
        if(!value) {
            updateSwitch();
            invoke("true");
        }
        value = true;
    }
    public void makeFalse() {
        invoke("false");
        value = false;
    }
    public void set(boolean newValue) {
        if(newValue) makeTrue(); else makeFalse();
    }
    public void setSwitch(boolean newValue) {
        if(switchVal != newValue) {
            invoke(newValue ? "switchTrue" : "switchFalse");
        }
        switchVal = newValue;
    }
    public boolean get() {
        return value;
    }
    public boolean getSwitch() {
        return switchVal;
    }
}
