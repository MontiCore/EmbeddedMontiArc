/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.visitor;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;

import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class MultiEnvironmentVisitorTests {
    File rootDirectory = new File("src/test/resources/visitor");
    File model = new File(rootDirectory, "NoAPTGetInstall.ddf");

    @Mock MultiEnvironmentVisitor visitor;

    @BeforeEach
    void before() {
        doCallRealMethod().when(visitor).handle(rootDirectory);
        doCallRealMethod().when(visitor).visit(rootDirectory);
        doCallRealMethod().when(visitor).endVisit(rootDirectory);
        doCallRealMethod().when(visitor).traverse(rootDirectory);
        doCallRealMethod().when(visitor).handleModel(model);
        doCallRealMethod().when(visitor).visitModel(model);
        doCallRealMethod().when(visitor).traverseModel(model);
        doCallRealMethod().when(visitor).endVisitModel(model);
    }

    @Test
    void testHandle() {
        visitor.handle(rootDirectory);

        verify(visitor).visit(rootDirectory);
        verify(visitor).traverse(rootDirectory);
        verify(visitor).endVisit(rootDirectory);
    }

    @Test
    void testVisit() {
        visitor.visit(rootDirectory);
    }

    @Test
    void testEndVisit() {
        visitor.endVisit(rootDirectory);
    }

    @Test
    void testTraverse() {
        visitor.traverse(rootDirectory);

        verify(visitor, times(5)).handleModel(any(File.class));
    }

    @Test
    void testHandleModel() {
        visitor.handleModel(model);

        verify(visitor).visitModel(model);
        verify(visitor).traverseModel(model);
        verify(visitor).endVisitModel(model);
    }

    @Test
    void testVisitModel() {
        visitor.visitModel(model);
    }

    @Test
    void testEndVisitModel() {
        visitor.endVisitModel(model);
    }

    @Test
    void testTraverseModel() {
        visitor.traverseModel(model);

        verify(visitor).handle(any(ASTEnvironmentCompilationUnit.class));
    }
}
