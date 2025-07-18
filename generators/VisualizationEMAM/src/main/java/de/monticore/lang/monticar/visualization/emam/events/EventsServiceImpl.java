/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.events;

import com.google.common.eventbus.EventBus;
import com.google.inject.Singleton;

@Singleton
public class EventsServiceImpl implements EventsService {
    protected final EventBus eventBus;

    public EventsServiceImpl() {
        this.eventBus = new EventBus();
    }

    @Override
    public void register(Object o) {
        this.eventBus.register(o);
    }

    @Override
    public void unregister(Object o) {
        this.eventBus.unregister(o);
    }

    @Override
    public void post(Object event) {
        this.eventBus.post(event);
    }
}
