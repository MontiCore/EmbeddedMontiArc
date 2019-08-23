/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.Simulation;

import org.apache.commons.math3.util.Pair;

public class Edge<T> extends Pair<T,T> {
    public Edge(T a, T b) {
        super(a, b);
    }

    @Override
    public boolean equals(Object o) {
        if(o == null){
            return false;
        }

        if(!(o instanceof Edge<?>)){
            return false;
        }else{
            Edge e = (Edge) o;
            return (this.getFirst().equals(e.getFirst()) && this.getSecond().equals(e.getSecond()))
                    || (this.getFirst().equals(e.getSecond()) && this.getSecond().equals(e.getFirst()));
        }
    }
}
