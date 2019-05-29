package Bus;


import commons.controller.commons.BusEntry;
import simulation.simulator.*;

import java.awt.*;
import java.util.*;
import java.lang.Math;


public class CANBus implements Bus {
    private int dataRate;// in bit/ms
    private Map<String, BusMessage> myMap;
    // BusMessage (also Object (message) und länge oder in der map ohne länge nur das objekt?

    /*
    static overhead per frame in bit
     */
    private final int OVERHEAD = 41;

    /*
    maximum payload per frame in byte
     */
    private final int MAX_PAYLOAD_LEN = 8;

    //dataRate in bit/ms
    //connections sind alle Anschlüsse an diesen Bus
    // map selber erstellen oder wo finden wir die?
    public CANBus(int dataRate, Optional[] connections) {
        this.dataRate = dataRate;
        //erstelle eigene map auf basis der gegebenen anschlüsse
        myMap = new HashMap<String, BusMessage>();
        for (int ix = 0; ix < connections.length; ix++) {
            myMap.put(busEntry.toString(connections[ix]), Optional.empty());
        }
    }

    public int setData(String key, BusMessage msg) {
        myMap.put(key, msg);
        //myMap.put(key, msg.getMessage);// falls nur objekt ohne länge
        int delay = getDelay(msg);
        return (delay);
    }

    public BusMessage getData(String key) {
        BusMessage data;
        // Object data;
        data = myMap.get(key);
        return data;
    }

    public Map<String, BusMessage> getAllData() {
        return myMap;
    }

    public String[] getImportNames() {
        Set keys = myMap.keySet();
        return (keys.toArray());
    }


    //gibt upload delay in ms
    private int getDelay(BusMessage msg) {
        int allData = msg.getMessageLen()*8 +(int) Math.ceil((double)msg.getMessageLen()/MAX_PAYLOAD_LEN)*OVERHEAD;
        double delay = allData/ dataRate;
        delay = Math.ceil(delay);
        return (int(delay));
    }
}