/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.models;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import org.apache.commons.cli.ParseException;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.List;
import java.util.logging.Logger;

@Singleton
public class ModelPathVisitorImpl extends SimpleFileVisitor<Path> implements ModelPathVisitor {
    protected final Logger logger;
    protected final ModelSplitter splitter;

    protected ModelsService registry;

    @Inject
    public ModelPathVisitorImpl(Logger logger, ModelSplitter splitter) {
        this.logger = logger;
        this.splitter = splitter;
    }

    @Override
    public void visit(Path modelPath, ModelsService registry) throws IOException {
        this.registry = registry;

        Files.walkFileTree(modelPath, this);
    }

    @Override
    public FileVisitResult visitFile(Path file, BasicFileAttributes attributes) throws IOException {
        if (file.toString().endsWith(".emam")) {
            String fileName = file.toString();

            this.logger.info("Splitting \"" + fileName + "\"...");

            List<File> models = this.splitter.split(file.toFile());

            this.logger.info("...\"" + fileName + "\" has been split.");
            this.registry.addAllModels(models);
        }

        return FileVisitResult.CONTINUE;
    }
}
