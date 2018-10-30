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
package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;

/**
 * @author Sascha Schneiders
 */
public class VariableType {
    protected String typeNameMontiCar;
    protected String typeNameTargetLanguage;
    protected String includeName = "";

    public VariableType() {

    }

    public VariableType(String typeNameMontiCar, String typeNameTargetLanguage, String includeName) {
        this.typeNameMontiCar = typeNameMontiCar;
        this.typeNameTargetLanguage = typeNameTargetLanguage;
        this.includeName = includeName;
    }

    public String getTypeNameMontiCar() {
        return typeNameMontiCar;
    }

    public void setTypeNameMontiCar(String typeNameMontiCar) {
        this.typeNameMontiCar = typeNameMontiCar;
    }

    public String getIncludeName() {
        return includeName;
    }

    public void setIncludeName(String includeName) {
        this.includeName = includeName;
    }

    public String getTypeNameTargetLanguage() {
        return typeNameTargetLanguage;
    }

    public void setTypeNameTargetLanguage(String typeNameTargetLanguage) {
        this.typeNameTargetLanguage = typeNameTargetLanguage;
    }

    public boolean hasInclude() {
        return !includeName.equals("");
    }

    public void set(VariableType variableType) {
        this.typeNameMontiCar = variableType.typeNameMontiCar;
        this.typeNameTargetLanguage = variableType.typeNameTargetLanguage;
        this.includeName = variableType.includeName;
    }
}
