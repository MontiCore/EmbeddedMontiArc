package de.rwth.montisim.simulation.commons;

import java.util.ArrayList;
import java.util.List;

public class Popper {
    protected transient List<Poppable> poppables = new ArrayList<>();

    public void addPoppable(Poppable p) {
        poppables.add(p);
    }

    public void performPop() {
        for (Poppable p : poppables) {
            p.pop();
        }
    }
}
