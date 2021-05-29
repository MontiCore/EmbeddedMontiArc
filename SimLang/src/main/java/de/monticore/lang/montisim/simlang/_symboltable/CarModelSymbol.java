/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.carlang.CarContainer;

public class CarModelSymbol extends CarModelSymbolTOP {

    private CarContainer model;

    public CarModelSymbol(String name) {
        super(name);
    }

    public void setCarContainer(CarContainer model) {
        this.model = model;
    }

    public CarContainer getCarContainer() {
        return model;
    }
}
