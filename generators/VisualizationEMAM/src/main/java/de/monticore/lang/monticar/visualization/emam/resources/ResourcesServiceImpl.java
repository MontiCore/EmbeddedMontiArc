/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.resources;

import de.monticore.lang.monticar.visualization.emam.application.Application;
import de.monticore.lang.monticar.visualization.emam.application.ApplicationContribution;
import org.apache.commons.io.FileUtils;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;

@Singleton
public class ResourcesServiceImpl implements ResourcesService, ApplicationContribution {
    protected final Logger logger;
    protected final Set<ResourcesContribution> contributions;
    protected final Map<String, String> resources;

    @Inject
    public ResourcesServiceImpl(Logger logger, Set<ResourcesContribution> contributions) {
        this.logger = logger;
        this.contributions = contributions;
        this.resources = new HashMap<>();
    }

    @Override
    public void configure(Application application) {
        this.logger.info("Adding Resources to ResourcesService...");

        for (ResourcesContribution contribution : this.contributions) {
            contribution.addToRegistry(this);
        }
    }

    @Override
    public boolean isPackaged() {
        return ResourcesServiceImpl.class.getResource("ResourcesServiceImpl.class").toString().startsWith("jar:");
    }

    @Override
    public void addResource(String id, String resource) {
        if (this.resources.containsKey(id)) this.logger.warning("A resource under the id of " + id + " was already registered");
        else this.resources.put(id, resource);
    }

    @Override
    public String getResource(String id) {
        return this.resources.get(id);
    }

    @Override
    public InputStream getResourceAsStream(String id) throws IOException {
        String resource = this.resources.get(id);

        if (this.isPackaged()) return this.getPackagedResourceAsStream(resource);
        else return this.getUnpackagedResourceAsStream(resource);
    }

    protected InputStream getPackagedResourceAsStream(String resource) {
        return ResourcesServiceImpl.class.getClassLoader().getResourceAsStream(resource);
    }

    protected InputStream getUnpackagedResourceAsStream(String resource) throws IOException {
        File resourceFile = new File("src/main/resources/" + resource);

        return FileUtils.openInputStream(resourceFile);
    }
}
