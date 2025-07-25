/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.helper;

import java.io.File;

public abstract class OrderableModelInfoCreator <T extends OrderableModelInfo> {

    public abstract T createNewInstance(String model);
    public abstract T createNewInstance(File modelFile);

}
