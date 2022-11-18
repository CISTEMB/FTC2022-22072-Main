package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

public class EventListener {
    private Map<String, Runnable> callbacks;

    public EventListener() {
        callbacks = new HashMap<>();
    }

    public void on(String id, Runnable callback) {
        callbacks.put(id, callback);
    }
    public void invoke(String id) {
        Runnable callback = callbacks.get(id);
        if(callback == null) return;

        callback.run();
    }
}
