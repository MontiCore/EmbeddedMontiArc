/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.visitor;

import de.monticore.lang.monticar.sol.grammars.environment._parser.EnvironmentParser;
import de.monticore.lang.monticar.sol.grammars.environment._visitor.EnvironmentVisitor;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.TrueFileFilter;
import org.apache.commons.io.filefilter.WildcardFileFilter;

import java.io.File;
import java.io.IOException;

public interface MultiEnvironmentVisitor extends EnvironmentVisitor {
    /**
     * Visitor Pattern 'handle' method for the root directory in which the models should be found and handled.
     * @param rootDirectory The root directory to be handled.
     */
    default void handle(File rootDirectory) {
        this.visit(rootDirectory);
        this.traverse(rootDirectory);
        this.endVisit(rootDirectory);
    }

    /**
     * A method which can be overwritten with custom functionality to be executed before the directory is traversed.
     * @param rootDirectory The directory which is visited.
     */
    default void visit(File rootDirectory) {}

    /**
     * A method which can be overwritten with custom functionality to be executed once the directory has been traversed.
     * @param rootDirectory The directory whose traversal has ended.
     */
    default void endVisit(File rootDirectory) {}

    /**
     * A method which triggers traversal of the root directory.
     * @param rootDirectory The directory to be traversed.
     */
    default void traverse(File rootDirectory) {
        WildcardFileFilter fileFilter = new WildcardFileFilter("*.ddf");

        FileUtils.listFiles(rootDirectory, fileFilter, TrueFileFilter.INSTANCE).forEach(this::handleModel);
    }

    /**
     * Visitor Pattern 'handle' method for the model to be handled.
     * @param model The model to be handled.
     */
    default void handleModel(File model) {
        this.visitModel(model);
        this.traverseModel(model);
        this.endVisitModel(model);
    }

    /**
     * A method which can be overwritten with custom functionality to be executed before the model is traversed.
     * @param model The model to be visited.
     */
    default void visitModel(File model) {}

    /**
     * A method which can be overwritten with custom functionality to be executed once the model has been traversed.
     * @param model The model whose traversal has ended.
     */
    default void endVisitModel(File model) {}

    /**
     * A method which triggers traversal of the model.
     * Traversal of the model consists of parsing and traversing the resulting AST.
     * @param model The model to be traversed.
     */
    default void traverseModel(File model) {
        EnvironmentParser parser = new EnvironmentParser();

        try {
            parser.parse(model.getPath()).ifPresent(this::handle);
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }
    }
}
