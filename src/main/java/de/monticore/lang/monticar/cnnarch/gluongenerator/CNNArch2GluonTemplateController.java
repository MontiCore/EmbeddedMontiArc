/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch._ast.ASTStream;
import de.monticore.lang.monticar.cnnarch.generator.ArchitectureElementData;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchTemplateController;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;

import java.io.Writer;
import java.util.*;

public class CNNArch2GluonTemplateController extends CNNArchTemplateController {
    public static final String NET_DEFINITION_MODE_KEY = "mode";

    public CNNArch2GluonTemplateController(ArchitectureSymbol architecture,
                                           TemplateConfiguration templateConfiguration) {
        super(architecture, templateConfiguration);
    }

    public void include(String relativePath, String templateWithoutFileEnding, Writer writer, NetDefinitionMode netDefinitionMode){
        String templatePath = relativePath + templateWithoutFileEnding + FTL_FILE_ENDING;
        Map<String, Object> ftlContext = new HashMap<>();
        ftlContext.put(TEMPLATE_CONTROLLER_KEY, this);
        ftlContext.put(ELEMENT_DATA_KEY, getCurrentElement());
        ftlContext.put(NET_DEFINITION_MODE_KEY, netDefinitionMode.toString());
        getTemplateConfiguration().processTemplate(ftlContext, templatePath, writer);
    }

    public void include(IOSymbol ioElement, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(ioElement);

        if (ioElement.isAtomic()){
            if (ioElement.isInput()){
                include(TEMPLATE_ELEMENTS_DIR_PATH, "Input", writer, netDefinitionMode);
            }
            else {
                include(TEMPLATE_ELEMENTS_DIR_PATH, "Output", writer, netDefinitionMode);
            }
        }
        else {
            include(ioElement.getResolvedThis().get(), writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(ConstantSymbol constant, Writer writer, NetDefinitionMode netDefinitionMode) {
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(constant);

        if (constant.isAtomic()) {
            include(TEMPLATE_ELEMENTS_DIR_PATH, "Const", writer, netDefinitionMode);
        }
        else {
            include(constant.getResolvedThis().get(), writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(LayerSymbol layer, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(layer);

        if (layer.isAtomic()){
            ArchitectureElementSymbol nextElement = layer.getOutputElement().get();
            String templateName = layer.getDeclaration().getName();
            include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer, netDefinitionMode);
        }
        else {
            include(layer.getResolvedThis().get(), writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(UnrollSymbol unrollElement, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(unrollElement);

        if(unrollElement.getDeclaration().getBody().getElements().get(0).isInput()) {
            include(unrollElement.getDeclaration().getBody().getElements().get(0).getResolvedThis().get(), writer, netDefinitionMode);
        }

        for(int i=0; i < (int)unrollElement.getDeclaration().getParameters().get(0).getExpression().getValue().get(); i++) {
            

            for (ArchitectureElementSymbol element : unrollElement.getDeclaration().getBody().getElements()) {
                previousElement = getCurrentElement();
                setCurrentElement(element);

                if (element.isAtomic() && !element.isInput() && !element.isOutput()) {
                    String templateName = element.getName();
                    include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer, netDefinitionMode);
                } else {
                    if(element.isOutput()) {
                        include(element.getResolvedThis().get(), writer, netDefinitionMode);
                    }
                }
            }
        }


        setCurrentElement(previousElement);
    }

    public void include(CompositeElementSymbol compositeElement, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(compositeElement);

        for (ArchitectureElementSymbol element : compositeElement.getElements()){
            include(element, writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(ArchitectureElementSymbol architectureElement, Writer writer, NetDefinitionMode netDefinitionMode){
        if (architectureElement instanceof CompositeElementSymbol){
            include((CompositeElementSymbol) architectureElement, writer, netDefinitionMode);
        }
        else if (architectureElement instanceof LayerSymbol){
            include((LayerSymbol) architectureElement, writer, netDefinitionMode);
        }
        else if (architectureElement instanceof UnrollSymbol){
            include((UnrollSymbol) architectureElement, writer, netDefinitionMode);
        }
        else if (architectureElement instanceof ConstantSymbol) {
            include((ConstantSymbol) architectureElement, writer, netDefinitionMode);
        }
        else {
            include((IOSymbol) architectureElement, writer, netDefinitionMode);
        }
    }

    public void include(ArchitectureElementSymbol architectureElementSymbol, String netDefinitionMode) {
        for(int i=0; i < ((ASTStream)architectureElementSymbol.getAstNode().get()).getElementsList().size(); i++){
            System.err.println(((ASTStream)architectureElementSymbol.getAstNode().get()).getElementsList().get(i).getSymbol().getName());
        }
        include(architectureElementSymbol, NetDefinitionMode.fromString(netDefinitionMode));
    }

    public void include(ArchitectureElementSymbol architectureElement, NetDefinitionMode netDefinitionMode){
        if (getWriter() == null){
            throw new IllegalStateException("missing writer");
        }
        include(architectureElement, getWriter(), netDefinitionMode);
    }

    public String ioNameToCpp(String ioName) {
        return ioName.replaceAll("_([0-9]+)_", "[$1]");
    }

    public List<String> getStreamInputNames(SerialCompositeElementSymbol stream) {
        List<String> names = new ArrayList<>();

        for (ArchitectureElementSymbol element : stream.getFirstAtomicElements()) {
            if(element instanceof  UnrollSymbol){
                for(ArchitectureElementSymbol sublayer: ((UnrollSymbol) element).getDeclaration().getBody().getFirstAtomicElements()){
                    names.add(getName(sublayer));
                }
            }else {
                names.add(getName(element));
            }
        }

        return names;
    }

    public List<String> getStreamOutputNames(SerialCompositeElementSymbol stream) {
        List<String> names = new ArrayList<>();

        for (ArchitectureElementSymbol element : stream.getLastAtomicElements()) {
            if(element instanceof  UnrollSymbol){
                for(ArchitectureElementSymbol sublayer: ((UnrollSymbol) element).getDeclaration().getBody().getLastAtomicElements()){
                    names.add(getName(sublayer));
                }
            }else {
                names.add(getName(element));
            }
        }

        return names;
    }
}
