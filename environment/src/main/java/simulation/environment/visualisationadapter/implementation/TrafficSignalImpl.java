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
package simulation.environment.visualisationadapter.implementation;

import java.time.Duration;

import com.google.gson.Gson;
import simulation.environment.visualisationadapter.interfaces.TrafficSignal;
import simulation.environment.visualisationadapter.interfaces.TrafficSignalStatus;
import simulation.util.Log;

/**
 * Created by Shahriar Robbani on 26.01.17.
 */
@Deprecated
public class TrafficSignalImpl implements TrafficSignal {

    private TrafficSignalStatus signalA = null;
    private TrafficSignalStatus signalB = null;
    private long timeDiffMs = 0;
    private boolean stateflag = true;

    public final static String EVENT_NEXT_SIMULATION_TIME_FRAME_TRAFIC_SIGNAL = "/event/nextTrafficSignalSimulationTimeFrame";

    public TrafficSignalImpl() {
        this.signalA = TrafficSignalStatus.GREEN;
        this.signalB = TrafficSignalStatus.RED;
    }

    @Override
    public TrafficSignalStatus getSignalA() {
        return this.signalA;
    }

    @Override
    public TrafficSignalStatus getSignalB() {
        return this.signalB;
    }

    @Override
    public void executeLoopIteration(Duration timeDiff) {
        this.timeDiffMs += timeDiff.toMillis();
        System.out.println(timeDiffMs);
        /*
         * try { Thread.sleep(timeDiffMs); } catch (InterruptedException e) {
         * e.printStackTrace(); }
         */
        if (this.timeDiffMs > 20000 && this.timeDiffMs < 30000) {
            if (this.signalA == TrafficSignalStatus.GREEN | this.signalA == TrafficSignalStatus.RED) {
                this.signalA = TrafficSignalStatus.YELLOW;
            }
            if (this.signalB == TrafficSignalStatus.RED | this.signalB == TrafficSignalStatus.GREEN) {
                this.signalB = TrafficSignalStatus.YELLOW;
            }
        }

        if (this.timeDiffMs > 30000) {
            this.timeDiffMs = 0;
            if (this.stateflag) {
                this.timeDiffMs = 0;
                if (this.signalA == TrafficSignalStatus.YELLOW) {
                    this.signalA = TrafficSignalStatus.RED;
                }
                if (this.signalB == TrafficSignalStatus.YELLOW) {
                    this.signalB = TrafficSignalStatus.GREEN;
                }
                flipFlag();
            } else {
                this.timeDiffMs = 0;
                if (this.signalA == TrafficSignalStatus.YELLOW) {
                    this.signalA = TrafficSignalStatus.GREEN;
                }
                if (this.signalB == TrafficSignalStatus.YELLOW) {
                    this.signalB = TrafficSignalStatus.RED;
                }
                flipFlag();
            }
        }
        Log.finest(this.timeDiffMs + "-" + this.signalA.toString() + ":" + this.signalB.toString());
    }

    private void flipFlag() {
        this.stateflag = !this.stateflag;
    }

    @Override
    public String toString() {
        Gson gson = new Gson();
        return gson.toJson(this);
    }
}