/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper;

import de.monticore.lang.monticar.generator.pythonwrapper.file.WrapperFileCreator;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.EmadlComponentTranslator;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.EmadlComponentTranslatorFactory;
import de.monticore.lang.monticar.generator.pythonwrapper.template.PythonWrapperTemplateControllerFactory;
import de.monticore.lang.monticar.generator.pythonwrapper.template.TemplateController;

/**
 *
 */
public class GeneratorPythonWrapperFactory {
    public GeneratorPythonWrapper create() {
        EmadlComponentTranslator componentTranslator = new EmadlComponentTranslatorFactory().create();
        TemplateController templateController = new PythonWrapperTemplateControllerFactory().create();
        WrapperFileCreator fileCreator = new WrapperFileCreator();
        return new GeneratorPythonWrapper(componentTranslator, templateController, fileCreator);
    }
}
