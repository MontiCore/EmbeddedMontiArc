/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport.helper;

import de.monticore.reporting.helper.OrderableModelInfo;
import de.monticore.reporting.order.ChildElement;
import java.lang.reflect.Field;

public class CheckCoCoResult extends OrderableModelInfo {

    public int componentCapitalized = 0;
    public int componentWithTypeParametersHasInstance = 0;
    public int connectorEndPointCorrectlyQualified = 0;
    public int defaultParametersHaveCorrectOrder = 0;
    public int inPortUniqueSender = 0;
    public int inRosPortRosSender;
    public int packageLowerCase = 0;
    public int parameterNamesUnique = 0;
    public int portTypeOnlyBooleanOrSIUnit = 0;
    public int portUsage = 0;
    public int referencedSubComponentExists = 0;
    public int simpleConnectorSourceExists = 0;
    public int sourceTargetNumberMatch = 0;
    public int subComponentsConnected = 0;
    public int topLevelComponentHasNoInstanceName = 0;
    public int typeParameterNamesUnique = 0;
    public int uniquePorts = 0;
    public int atomicComponentCoCo = 0;
    public int matrixAssignmentDeclarationCheck = 0;
    public int matrixAssignmentCheck = 0;
    public int onlyIncomingPortIsConfig = 0;
    public int dynamicComponentDynamicBodyElements = 0;
    public int noDynamicNewComponentAndPort = 0;
    public int noDynamicNewConnectsOutsideEventHandler = 0;
    public int referencedSubComponentExistsEMAM = 0;
    public int checkLayer = 0;
    public int checkRangeOperators = 0;
    public int checkVariableName = 0;
    public int checkLayerName = 0;
    public int checkArgument = 0;
    public int checkLayerRecursion = 0;
    public int checkBehaviorName = 0;

    public CheckCoCoResult(String pathToFile) {
        super(pathToFile);
    }

    public String toString() {
        return this.getModelName();
    }

    public boolean isValid() {
        Field[] fields = this.getClass().getDeclaredFields();
        boolean res = true;
        for (Field field: fields) {
            try {
                if (((int) field.get(this)) < 0)
                    res = false;
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        return res && getResolved() >= 0 && getParsed() >= 0;
    }

    public Data[] getFieldsAndValues() {
        Field[] fields = this.getClass().getDeclaredFields();
        Data[] data = new Data[fields.length];
        for (int i = 0; i < fields.length; i++) {
            Field field = fields[i];
            try {
                data[i] = new Data(field.getName(), (Integer) field.get(this));
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        return data;
    }

    public class Data {
        public String name;
        public int value;

        public Data(String name, int value) {
            this.name = name;
            this.value = value;
        }

        public String getQuotedName() {
            String Name = this.name.substring(0, 1).toUpperCase() + this.name.substring(1);
            return "\"" + Name + "\"";
        }
    }

    @Override
    public String getErrorMessage() {
        if (isErrorResult() || isMainPackage()) return "";

        String msg = "";

        for (String m : getErrorMessages()) {
            if (m != null) {
                String ms = m.replace("\r\n", "<br>")
                        .replace("\n", "<br>")
                        .replace("\r", "<br>")
                        .replace("\t", "&#9;")
                        .replace("\"", "&quot;")
                        .replace("\'", "&prime;")
                        .replace("\\", "&bsol;")
                        .replace("\0", "")
                        + "<br>";
                if (!ms.contains("[ERROR]") && !ms.contains("[INFO]") && !ms.contains("[WARNING]"))
                    ms = "[ERROR] " + ms;
                msg += ms;
            }
        }
        msg = msg.substring(0, msg.length() - 4);

        return msg;
    }

    @Override
    public void setModelPath(String modelPath) {
        String res = modelPath.replace("\\", "/");
        if (res.charAt(res.length() - 1) != '/')
            res = res + "/";
        super.setModelPath(res);
    }

    @Override
    public void setProject(String project) {
        String res = project.replace("\\", "/");
        if (res.charAt(res.length() - 1) != '/')
            res = res + "/";
        super.setProject(res);
    }

    public void setChildInfo() {
        CheckCoCoResult mainPackage = this;
        Field[] fields = this.getClass().getDeclaredFields();
        for (ChildElement childElement : getChildren()) {
            CheckCoCoResult child = (CheckCoCoResult) childElement.getChild();

            for (Field field: fields) {
                int childVal = 0;
                int mainVal = 0;
                try {
                    childVal = (int) field.get(child);
                    mainVal = (int) field.get(mainPackage);
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
                if (childVal != 0 && childVal < mainVal || mainVal == 0) {
                    try {
                        field.set(mainPackage, childVal);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }
}
