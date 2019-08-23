/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.events;

public interface EventsService {
    void register(Object o);
    void unregister(Object o);
    void post(Object event);
}
