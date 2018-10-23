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
package simulation.network.tasks;

import simulation.network.NetworkDiscreteEvent;
import simulation.network.NetworkNode;
import simulation.network.NetworkTask;
import simulation.network.NetworkTaskId;
import simulation.util.Log;
import java.util.LinkedList;

/**
 * Empty network task that does nothing, should not be used regularly
 */
public class TaskNone extends NetworkTask {

    /**
     * Constructor for this task
     *
     * @param node Node that the task is created for
     */
    public TaskNone(NetworkNode node) {
        Log.warning("TaskNone: Constructor - This class should not be used regularly, Node: " + node);
        setTaskId(NetworkTaskId.NETWORK_TASK_ID_NONE);
        setNetworkNode(node);
        setTaskEventIdList(new LinkedList<>());
    }

    /**
     * Function that handles network events internally in the task
     *
     * @param event Network discrete event to be handled
     */
    @Override
    public void taskHandleNetworkEvent(NetworkDiscreteEvent event) {
        // Do nothing
    }
}