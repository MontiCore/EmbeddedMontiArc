/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.events;

import com.google.inject.AbstractModule;

public class EventsModule extends AbstractModule {
    @Override
    public void configure() {
        bind(EventsService.class).to(EventsServiceImpl.class);
    }
}
