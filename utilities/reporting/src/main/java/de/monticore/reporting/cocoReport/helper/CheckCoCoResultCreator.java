/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport.helper;

import de.monticore.reporting.helper.OrderableModelInfo;
import de.monticore.reporting.helper.OrderableModelInfoCreator;

import java.io.File;

public class CheckCoCoResultCreator extends OrderableModelInfoCreator{

    @Override
    public OrderableModelInfo createNewInstance(String model) {
        return new CheckCoCoResult(model);
    }

    @Override
    public OrderableModelInfo createNewInstance(File modelFile) {
        return new CheckCoCoResult(modelFile.getAbsolutePath());
    }
}
