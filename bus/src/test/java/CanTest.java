/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
