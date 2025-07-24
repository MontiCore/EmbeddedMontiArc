/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppVariableViewModel;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppWrapperViewModel;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.io.Writer;
import java.util.Map;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.entry;
import static org.mockito.Mockito.*;

/**
 *
 */
public class TemplateControllerTest {
    private final static CppWrapperViewModel ANY_VIEW_MODEL = new CppWrapperViewModel(
            "any_component_executor",
            "any_component",
            "any_component_input",
            "any_component_output",
            Lists.newArrayList(new CppVariableViewModel("int", "anyInput")),
            Lists.newArrayList(new CppVariableViewModel("double", "anyOutput"))
    );
    private final static ComponentPortInformation ANY_COMPONENT_PORT_INFORMATION = new ComponentPortInformation("any_component");

    private TemplateController templateController;

    @Before
    public void setup() throws IOException, TemplateException {
        ANY_COMPONENT_PORT_INFORMATION.addInput(PortVariable.primitiveVariableFrom("anyVar", EmadlType.Z, PortDirection.INPUT));
        ANY_COMPONENT_PORT_INFORMATION.addOutput(PortVariable.primitiveVariableFrom("anyVar2", EmadlType.Q, PortDirection.OUTPUT));


        TemplateLoader templateLoader = mock(TemplateLoader.class, RETURNS_DEEP_STUBS);
        TemplateDataPreparer templateDataPreparer = mock(TemplateDataPreparer.class, RETURNS_DEEP_STUBS);

        when(templateDataPreparer.getWrapperViewModel(ANY_COMPONENT_PORT_INFORMATION)).thenReturn(ANY_VIEW_MODEL);

        doAnswer(invocation -> {
            Writer w = (Writer)invocation.getArguments()[1];
            w.write("CPP");
            return null;
        }).when(templateLoader).processWrapperCppTemplate(eq(ANY_VIEW_MODEL), any());

        doAnswer(invocation -> {
            Writer w = (Writer)invocation.getArguments()[1];
            w.write("HEADER");
            return null;
        }).when(templateLoader).processWrapperHeaderTemplate(eq(ANY_VIEW_MODEL), any());

        doAnswer(invocation -> {
            Writer w = (Writer)invocation.getArguments()[1];
            w.write("INTERFACE FILE");
            return null;
        }).when(templateLoader).processSwigTemplate(eq(ANY_VIEW_MODEL), any());

        doAnswer(invocation -> {
            Writer w = (Writer)invocation.getArguments()[1];
            w.write("CMAKE FILE");
            return null;
        }).when(templateLoader).processCmakeTemplate(eq(ANY_VIEW_MODEL), any());

        this.templateController = new TemplateController(templateLoader, templateDataPreparer);
    }

    @Test
    public void testTemplateDataModelIsPreparedAndTemplateLoaderIsCalled() {
        // when
        Map<String, String> resultFiles = this.templateController.processWrapper(ANY_COMPONENT_PORT_INFORMATION);

        // then
        assertThat(resultFiles).isNotNull().isNotEmpty().hasSize(4);
        assertThat(resultFiles).contains(entry("any_component_executor.h", "HEADER"));
        assertThat(resultFiles).contains(entry("any_component_executor.cpp", "CPP"));
        assertThat(resultFiles).contains(entry("any_component_executor.i", "INTERFACE FILE"));
        assertThat(resultFiles).contains(entry("CMakeLists.txt", "CMAKE FILE"));
    }
}
