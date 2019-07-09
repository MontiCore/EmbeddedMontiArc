package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.generator.ArchitectureElementData;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchTemplateController;
import de.monticore.lang.monticar.cnnarch._symboltable.*;

import java.io.Writer;

public class CNNArch2MxNetTemplateController extends CNNArchTemplateController {

    public CNNArch2MxNetTemplateController(ArchitectureSymbol architecture) {
        super(architecture, new MxNetTemplateConfiguration());
    }

    public void include(IOSymbol ioElement, Writer writer){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(ioElement);

        if (ioElement.isAtomic()){
            if (ioElement.isInput()){
                include(TEMPLATE_ELEMENTS_DIR_PATH, "Input", writer);
            } else {
                include(TEMPLATE_ELEMENTS_DIR_PATH, "Output", writer);
            }
        } else {
            include(ioElement.getResolvedThis().get(), writer);
        }

        setCurrentElement(previousElement);
    }

    public void include(LayerSymbol layer, Writer writer){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(layer);

        if (layer.isAtomic()){
            ArchitectureElementSymbol nextElement = layer.getOutputElement().get();
                String templateName = layer.getDeclaration().getName();
                include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer);
        } else {
            include(layer.getResolvedThis().get(), writer);
        }

        setCurrentElement(previousElement);
    }

    public void include(CompositeElementSymbol compositeElement, Writer writer){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(compositeElement);

        for (ArchitectureElementSymbol element : compositeElement.getElements()){
            include(element, writer);
        }

        setCurrentElement(previousElement);
    }

    public void include(ArchitectureElementSymbol architectureElement, Writer writer){
        if (architectureElement instanceof CompositeElementSymbol){
            include((CompositeElementSymbol) architectureElement, writer);
        } else if (architectureElement instanceof LayerSymbol) {
            include((LayerSymbol) architectureElement, writer);
        } else {
            include((IOSymbol) architectureElement, writer);
        }
    }

    public void include(ArchitectureElementSymbol architectureElement){
        if (getWriter() == null){
            throw new IllegalStateException("missing writer");
        }
        include(architectureElement, getWriter());
    }
}