/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.models;

import com.google.common.eventbus.Subscribe;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.visualization.emam.application.Application;
import de.monticore.lang.monticar.visualization.emam.application.ApplicationContribution;
import de.monticore.lang.monticar.visualization.emam.events.EventsService;
import de.monticore.lang.monticar.visualization.emam.options.OptionsParsedEvent;
import org.apache.commons.cli.ParseException;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.logging.Logger;

@Singleton
public class ModelsServiceImpl implements ModelsService, ApplicationContribution {
    protected final EventsService eventsService;
    protected final ModelPathVisitor visitor;
    protected final List<File> models;

    @Inject
    public ModelsServiceImpl(Logger logger, EventsService eventsService, ModelPathVisitor visitor) {
        this.eventsService = eventsService;
        this.visitor = visitor;
        this.models = new ArrayList<>();
    }

    @Override
    public void addModel(File model) {
        this.models.add(model);
    }

    @Override
    public void addAllModels(Collection<File> models) {
        this.models.addAll(models);
    }

    @Override
    public List<File> getModels() {
        return new ArrayList<>(this.models);
    }

    @Override
    public void clearModels() {
        this.models.clear();
    }

    @Override
    public void prepare(Application application) {
        this.eventsService.register(this);
    }

    @Override
    public void stop(Application application) throws IOException {
        for (File model : this.models) {
            FileUtils.forceDelete(model);
        }
    }

    @Subscribe
    public void onOptionsParsed(OptionsParsedEvent event) throws ParseException, IOException {
        Path modelPath = event.optionsService.getOptionAsPath("mp");
        ModelPathVisitedEvent visitedEvent = new ModelPathVisitedEvent(this);

        this.visitor.visit(modelPath, this);
        this.eventsService.post(visitedEvent);
    }
}
