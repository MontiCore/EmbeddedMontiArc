/**
 *
 *  ******************************************************************************
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
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator;

import org.junit.Test;
import org.junit.Assert;

import java.io.File;
import java.io.Serializable;
import java.util.HashMap;

public class HardwareEmulatorTest {
    @Test
    public void basic_test() throws Exception {
        HardwareEmulatorInterface manager;
        try {
            manager = new HardwareEmulatorInterface("autopilots_folder=autopilots", "");
        } catch(Exception e){
            e.printStackTrace();
            throw new Exception("TEST ERROR IN HARDWARE EMULATOR");
        }

        String querry = "get_available_autopilots\nget_available_threads";
        String res = manager.query(querry);
        System.out.println("EmulatorManager querry response: " + res);

        String lines[] = res.split("\n");
        Assert.assertEquals("Querry result line count", 2, lines.length);
        String line1[] = lines[0].split("=");
        Assert.assertEquals("available_autopilots two values (=)", 2, line1.length);
        Assert.assertEquals("available_autopilots command", new String("available_autopilots"), line1[0]);

        String line2[] = lines[1].split("=");
        Assert.assertEquals("available_threads two values (=)", 2, line2.length);
        Assert.assertEquals("available_threads command", new String("available_threads"), line2[0]);

        String config = "autopilot=AutopilotAdapter";
        config += "\nos=windows";
        int id = manager.alloc_autopilot(config);
        if (id < 0){
            String q = manager.query( "get_error_msg" );
            Assert.assertTrue("Could not allocate Emulator: " + q, id >= 0);
        }

        HashMap<String, Serializable> inputs = new HashMap<String, Serializable>();
        inputs.put("timeIncrement", 1.0);
        inputs.put("currentVelocity", 0.0);
        inputs.put("x", 0.01);
        inputs.put("y", 0.01);
        inputs.put("compass", 0.0);
        inputs.put("currentEngine", 0.0);
        inputs.put("currentSteering", 0.0);
        inputs.put("currentBrakes", 0.0);
        inputs.put("trajectory_length", 5);
        inputs.put("trajectory_x", new double[]{0.01, 0.02, 0.03, 0.04, 0.05, 0.06});
        inputs.put("trajectory_y", new double[]{0.01, 0.01, 0.02, 0.02, 0.01, 0.01});
        manager.update_bus(id, inputs);
        //manager.start_tick(1000000);
        //manager.end_tick();
        HashMap<String, Serializable> outputs = manager.old_execute(id, 1000000, inputs);
        String emu_querry = manager.query_autopilot(id, "get_avg_runtime");
        System.out.println("Emu querry: " + emu_querry);
        //HashMap<String, Serializable> outputs = manager.get_outputs(id);
        Assert.assertEquals("output count", outputs.size(), 3);
        for (HashMap.Entry<String, Serializable> entry : outputs.entrySet()) {
            System.out.println(entry.getKey() + " = " + entry.getValue());
        }
        manager.free_autopilot(id);

        //Try to clear up the library from the test directory
        File lib = manager.get_library();
        manager = null;
        System.gc();
        lib.delete();
    }
}
