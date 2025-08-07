/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct.coco;

import de.monticore.lang.monticar.struct._cocos.StructCoCoChecker;

public class DefaultStructCoCoChecker {
    public static StructCoCoChecker create() {
        return new StructCoCoChecker()
                .addCoCo(new NoRecursiveStructReferences())
                .addCoCo(new PrimitiveTypesAreForbidden())
                .addCoCo(new PrimitiveArrayTypesAreForbidden())
                .addCoCo(new StructCapitalized())
                .addCoCo(new StructFieldsHaveUniqueNames())
                .addCoCo(new GenericsAreForbidden())
                .addCoCo(new ComplexTypesAreForbidden());
    }
}
