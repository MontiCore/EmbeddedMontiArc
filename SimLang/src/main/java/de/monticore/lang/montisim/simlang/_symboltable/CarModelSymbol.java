package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.carlang.CarModel;

public class CarModelSymbol extends CarModelSymbolTOP {

    private CarModel model;

    public CarModelSymbol(String name) {
        super(name);
    }

    public void setCarModel(CarModel model) {
        this.model = model;
    }

    public CarModel getCarModel() {
        return model;
    }
}
