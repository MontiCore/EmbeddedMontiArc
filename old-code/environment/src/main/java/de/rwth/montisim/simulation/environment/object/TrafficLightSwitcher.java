/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.object;

import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.simulation.environment.visualisationadapter.SignTypeAndState;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by lukas on 10.03.17.
 */
public class TrafficLightSwitcher implements Updatable {

    private static List<TrafficLightSwitcher> switcher = new ArrayList<>();

    public static List<TrafficLightSwitcher> getSwitcher() {
        return switcher;
    }

    public static void addSwitcher(TrafficLightSwitcher switcherInst) {
        switcher.add(switcherInst);
    }

    // TODO
    // private long time;

    // private List<TrafficLight> signals;

    // private int currentIndex;

    // private List<Long> changedState;

    // public TrafficLightSwitcher(List<TrafficLight> signals) {
    //     this.signals = signals;
    //     this.time = 0l;
    //     this.currentIndex = 0;
    //     this.changedState = new ArrayList<>();
    //     initSignalStates();
    // }

    // private void initSignalStates() {
    //     for(int i = 0; i < signals.size(); i++) {
    //         if(i == currentIndex) {
    //             signals.get(i).setState(SignTypeAndState.TRAFFIC_LIGHT_GREEN);
    //         } else {
    //             signals.get(i).setState(SignTypeAndState.TRAFFIC_LIGHT_RED);
    //         }
    //     }
    // }

    // public List<Long> getChangedState() {
    //     return this.changedState;
    // }

    @Override
    public void update(Duration deltaT) {
        // this.changedState.clear();
        // this.time += deltaT.toMillis();
        // if(time >= 30000 && time <= 40000) {
        //     this.signals.get(currentIndex).setState(SignTypeAndState.TRAFFIC_LIGHT_YELLOW);
        //     this.changedState.add(this.signals.get(currentIndex).getId());
        // }

        // if(time > 40000) {
        //     this.signals.get(currentIndex).setState(SignTypeAndState.TRAFFIC_LIGHT_RED);
        //     this.changedState.add(this.signals.get(currentIndex).getId());

        //     currentIndex++;
        //     currentIndex = currentIndex % this.signals.size();


        //     this.signals.get(currentIndex).setState(SignTypeAndState.TRAFFIC_LIGHT_RED_YELLOW);
        //     this.changedState.add(this.signals.get(currentIndex).getId());
        //     this.time = 0;
        // }

        // if(time > 10000 && time < 30000) {
        //     this.signals.get(currentIndex).setState(SignTypeAndState.TRAFFIC_LIGHT_GREEN);
        //     this.changedState.add(this.signals.get(currentIndex).getId());
        // }
    }
}
