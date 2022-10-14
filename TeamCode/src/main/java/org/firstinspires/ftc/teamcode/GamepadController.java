package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadController {
    private Runnable callbackDownA;
    private Runnable callbackDownB;
    private Runnable callbackDownX;
    private Runnable callbackDownY;

    private Runnable callbackDownDU;
    private Runnable callbackDownDD;
    private Runnable callbackDownDL;
    private Runnable callbackDownDR;

    private Gamepad gamepad;

    public GamepadController(Gamepad newGamepad) {
        gamepad = newGamepad;
    }

    public void pressButtonA(Runnable callback) {
        callbackDownA = callback;
    }
    public void pressButtonB(Runnable callback) {
        callbackDownB = callback;
    }
    public void pressButtonX(Runnable callback) {
        callbackDownX = callback;
    }
    public void pressButtonY(Runnable callback) {
        callbackDownY = callback;
    }

    public void pressButtonDU(Runnable callback) {
        callbackDownDU = callback;
    }
    public void pressButtonDD(Runnable callback) {
        callbackDownDD = callback;
    }
    public void pressButtonDL(Runnable callback) {
        callbackDownDL = callback;
    }
    public void pressButtonDR(Runnable callback) {
        callbackDownDR = callback;
    }

    private boolean a = false;
    private boolean b = false;
    private boolean x = false;
    private boolean y = false;
    private boolean du = false;
    private boolean dd = false;
    private boolean dl = false;
    private boolean dr = false;
    public void update() {
        if(gamepad.a) {
            if(!a) {
                if (callbackDownA != null) callbackDownA.run();
                a = true;
            }
        } else {
            a = false;
        }

        if(gamepad.b) {
            if(!b) {
                if (callbackDownB != null) callbackDownB.run();
                b = true;
            }
        } else {
            b = false;
        }

        if(gamepad.x) {
            if(!x) {
                if (callbackDownX != null) callbackDownX.run();
                x = true;
            }
        } else {
            x = false;
        }

        if(gamepad.y) {
            if(!y) {
                if (callbackDownY != null) callbackDownY.run();
                y = true;
            }
        } else {
            y = false;
        }

        if(gamepad.dpad_up) {
            if(!du) {
                if (callbackDownDU != null) callbackDownDU.run();
                du = true;
            }
        } else {
            du = false;
        }

        if(gamepad.dpad_down) {
            if(!dd) {
                if (callbackDownDD != null) callbackDownDD.run();
                dd = true;
            }
        } else {
            dd = false;
        }

        if(gamepad.dpad_left) {
            if(!dl) {
                if (callbackDownDL != null) callbackDownDL.run();
                dl = true;
            }
        } else {
            dl = false;
        }

        if(gamepad.dpad_right) {
            if(!dr) {
                if (callbackDownDR != null) callbackDownDR.run();
                dr = true;
            }
        } else {
            dr = false;
        }
    }

}
