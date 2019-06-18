//import static org.junit.Assert.assertEquals;
//
//
//import java.util.ArrayList;
//import java.util.List;
//
//import org.junit.Test;
//
//
//public class CanTest {
//
//
//    //@Test
//    public void testDelay() {
//        int dataRate = 3;
//        CanBus bus= new CanBus(dataRate);
//        BusMessage msg = new BusMessage(null, 5, BusEntry.SENSOR_LANE, 15, 3);
//        int expected = (int)Math.ceil((5*8+41)/dataRate);
//        assertEquals(expected, bus.getDelay(msg));
//
//    }
//
//    //@Test
//    public void testNextMessage(){
//        int dataRate = 3;
//        CanBus bus = new CanBus(dataRate);
//        BusMessage msg1 = new BusMessage(null, 6, BusEntry.SENSOR_LANE, 18, 4);
//        BusMessage msg2 = new BusMessage(null, 5, BusEntry.SENSOR_CAMERA, 12, 3);
//        BusMessage msg3 = new BusMessage(null, 7, BusEntry.SENSOR_WEATHER, 15, 1);
//
//        BusMessage expected = msg2;
//        // funktion aufrufen geht nicht, da activeMessage leer ist
//
//    }
//
//
//}
