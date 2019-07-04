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
package simulation.environment.object;

import commons.simulation.SimulationLoopExecutable;
import simulation.environment.visualisationadapter.implementation.TrafficLight;
import simulation.environment.visualisationadapter.interfaces.SignTypeAndState;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by lukas on 10.03.17.
 */
public class TrafficLightSwitcher implements SimulationLoopExecutable{

    private static List<TrafficLightSwitcher> switcher = new ArrayList<>();

    public static List<TrafficLightSwitcher> getSwitcher() {
        return switcher;
    }

    public static void addSwitcher(TrafficLightSwitcher switcherInst) {
        switcher.add(switcherInst);
    }


    private long time;

    private List<TrafficLight> signals;

    private int currentIndex;

    private List<Long> changedState;

    public TrafficLightSwitcher(List<TrafficLight> signals) {
        this.signals = signals;
        this.time = 0l;
        this.currentIndex = 0;
        this.changedState = new ArrayList<>();
        initSignalStates();
    }

    private void initSignalStates() {
        for(int i = 0; i < signals.size(); i++) {
            if(i == currentIndex) {
                signals.get(i).setState(SignTypeAndState.TRAFFIC_LIGHT_GREEN);
            } else {
                signals.get(i).setState(SignTypeAndState.TRAFFIC_LIGHT_RED);
            }
        }
    }


    @Override
    public void executeLoopIteration(Duration timeDiff) {
        this.changedState.clear();
        this.time += timeDiff.toMillis();
        if(time >= 30000 && time <= 40000) {
            this.signals.get(currentIndex).setState(SignTypeAndState.TRAFFIC_LIGHT_YELLOW);
            this.changedState.add(this.signals.get(currentIndex).getId());
        }

        if(time > 40000) {
            this.signals.get(currentIndex).setState(SignTypeAndState.TRAFFIC_LIGHT_RED);
            this.changedState.add(this.signals.get(currentIndex).getId());

            currentIndex++;
            currentIndex = currentIndex % this.signals.size();


            this.signals.get(currentIndex).setState(SignTypeAndState.TRAFFIC_LIGHT_RED_YELLOW);
            this.changedState.add(this.signals.get(currentIndex).getId());
            this.time = 0;
        }

        if(time > 10000 && time < 30000) {
            this.signals.get(currentIndex).setState(SignTypeAndState.TRAFFIC_LIGHT_GREEN);
            this.changedState.add(this.signals.get(currentIndex).getId());
        }
    }

    public List<Long> getChangedState() {
        return this.changedState;
    }
}