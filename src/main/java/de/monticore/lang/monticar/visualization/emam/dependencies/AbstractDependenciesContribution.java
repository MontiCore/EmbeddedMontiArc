/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.dependencies;

import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.paths.PathsService;
import de.monticore.lang.monticar.visualization.emam.resources.ResourcesService;
import org.apache.commons.codec.digest.DigestUtils;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.security.MessageDigest;
import java.util.Arrays;
import java.util.logging.Logger;

public abstract class AbstractDependenciesContribution implements DependenciesContribution {
    protected final Logger logger;
    protected final PathsService pathsService;
    protected final ResourcesService resourcesService;

    @Inject
    public AbstractDependenciesContribution(Logger logger, PathsService pathsService, ResourcesService resourcesService) {
        this.logger = logger;
        this.pathsService = pathsService;
        this.resourcesService = resourcesService;
    }

    protected boolean isInstalled(String id) throws Exception {
        MessageDigest messageDigest = MessageDigest.getInstance("MD5");
        DigestUtils digestUtils = new DigestUtils(messageDigest);

        InputStream toInstall = this.resourcesService.getResourceAsStream(id);
        File installed = this.pathsService.getPathAsFile(id);

        return installed.exists() && Arrays.equals(digestUtils.digest(toInstall), digestUtils.digest(installed));
    }

    protected void install(String id) throws IOException {
        InputStream toInstall = this.resourcesService.getResourceAsStream(id);
        File destination = this.pathsService.getPathAsFile(id);

        FileUtils.copyInputStreamToFile(toInstall, destination);
    }
}
