package simulation.environment.object;

import commons.simulation.SimulationLoopExecutable;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import simulation.environment.util.ChargingProcess;
import simulation.util.Log;
import simulation.environment.object.Battery;

import java.util.*;

public class BatteryPrototype implements SimulationLoopExecutable, Battery{
    double ampere;
    double voltage;
    double percentage;
    boolean isCharging;


    public BatteryPrototype() {
        percentage = 100;

    }

    public void executeLoopIteration(long timeDiffMs) {

        if (timeDiffMs > 1000 * 2) {
            if(isCharging = false){
                percentage = percentage - 5;
            }
        }
    }

    public void recharge (){
        percentage = percentage + 20;
    }

    @java.lang.Override
    public int getStateinPercentage() {
        return 0;
    }

    @java.lang.Override
    public boolean isFullyCharged() {
        return false;
    }
}
