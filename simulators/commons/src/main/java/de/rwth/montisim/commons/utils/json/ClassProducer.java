/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils.json;

public interface ClassProducer<T> {
    Class<? extends T> getClass(String type);
}
