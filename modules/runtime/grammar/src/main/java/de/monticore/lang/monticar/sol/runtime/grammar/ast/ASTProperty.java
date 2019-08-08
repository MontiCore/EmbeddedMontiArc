/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.runtime.grammar.ast;

import de.monticore.ast.ASTNode;

import java.util.HashMap;
import java.util.Map;

/**
 * This class is based on ANTLR's ParseTreeProperty class.
 */
public class ASTProperty<T> {
    protected final Map<ASTNode, T> annotations;

    public ASTProperty() {
        this.annotations = new HashMap<>();
    }

    public T get(ASTNode node) {
        return this.annotations.get(node);
    }

    public void put(ASTNode node, T value) {
        this.annotations.put(node, value);
    }

    public T removeFrom(ASTNode node) {
        return this.annotations.remove(node);
    }

    public String toString() {
        return this.annotations.toString();
    }
}
