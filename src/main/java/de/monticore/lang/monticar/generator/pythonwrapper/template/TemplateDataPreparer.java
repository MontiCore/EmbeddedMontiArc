/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template;

import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDimensionType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppVariableViewModel;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppWrapperViewModel;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 *
 */
class TemplateDataPreparer {
    private static final String ARMA_NAMESPACE = "arma::";

    public CppWrapperViewModel getWrapperViewModel(final ComponentPortInformation componentPortInformation) {
        checkNotNull(componentPortInformation);
        checkNotNull(componentPortInformation.getComponentName());
        checkNotNull(componentPortInformation.getAllInputs());
        checkNotNull(componentPortInformation.getAllOutputs());

        final String wrapperClassName = componentPortInformation.getComponentName() + "_executor";
        final String wrappedClassName = componentPortInformation.getComponentName();
        final String inputClassName = componentPortInformation.getComponentName() + "_input";
        final String outputClassName = componentPortInformation.getComponentName() + "_output";
        final List<CppVariableViewModel> inputVariables = componentPortInformation.getAllInputs()
                .stream()
                .map(this::getVariableViewModel)
                .collect(Collectors.toList());
        final List<CppVariableViewModel> outputVariables = componentPortInformation.getAllOutputs()
                .stream()
                .map(this::getVariableViewModel)
                .collect(Collectors.toList());

        return new CppWrapperViewModel(wrapperClassName, wrappedClassName, inputClassName, outputClassName,
                inputVariables, outputVariables);
    }

    public CppVariableViewModel getVariableViewModel(final PortVariable portVariable) {
        checkNotNull(portVariable);
        checkNotNull(portVariable.getPortDirection());
        checkNotNull(portVariable.getPortDimensionType());
        checkNotNull(portVariable.getDimension());
        checkNotNull(portVariable.getVariableName());
        String cppTypeName = getCppTypeName(portVariable);
        return new CppVariableViewModel(cppTypeName, portVariable.getVariableName());
    }

    private String getCppTypeName(PortVariable portVariable) {
        String cppTypeName = "";
        if (portVariable.getPortDimensionType().equals(PortDimensionType.PRIMITIVE)) {
            cppTypeName = getPrimitiveCppTypeName(portVariable.getEmadlType());
        } else if (portVariable.getPortDimensionType().equals(PortDimensionType.MULTIDIMENSIONAL)) {
            cppTypeName = getMultidimensionalCppTypeName(portVariable.getDimension().size(), portVariable.getEmadlType());
        }
        return cppTypeName;
    }

    private String getMultidimensionalCppTypeName(Integer dimensionSize, EmadlType emadlType) {
        String prefix = "";
        String type = "";
        if (dimensionSize == 1) {
            type = "vec";
        } else if(dimensionSize == 2) {
            type = "mat";
        } else if(dimensionSize == 3) {
            type = "cube";
        } else {
            logAndThrowViewCreationException("Dimension size is greater 3 and not supported as template data");
        }

        switch(emadlType) {
            case Q:
                prefix = (type.equals("vec") ? "col" : "");
                break;
            case Z:
                prefix = "i";
                break;
            default:
                logAndThrowViewCreationException("Unknown EMADL type. Cannot create template data");
        }
        return ARMA_NAMESPACE + prefix + type;
    }

    private String getPrimitiveCppTypeName(EmadlType emadlType) {
        String cppTypeName = "";
        switch(emadlType) {
            case Q:
                cppTypeName = "double";
                break;
            case Z:
                cppTypeName = "int";
                break;
            case B:
                cppTypeName = "bool";
                break;
            default:
                logAndThrowViewCreationException("Unknown EMADL type. Cannot create template data");
        }
        return cppTypeName;
    }


    private void logAndThrowViewCreationException(String s) {
        Log.error(s);
        throw new ViewCreationException(s);
    }
}
