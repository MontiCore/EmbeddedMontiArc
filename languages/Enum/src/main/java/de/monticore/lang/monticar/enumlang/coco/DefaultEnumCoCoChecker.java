/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.enumlang.coco;

import de.monticore.lang.monticar.enumlang._cocos.EnumLangCoCoChecker;

public final class DefaultEnumCoCoChecker {

    private DefaultEnumCoCoChecker() {
        // utility class
    }

    public static EnumLangCoCoChecker create() {
        return new EnumLangCoCoChecker()
                .addCoCo(new EnumCapitalized())
                .addCoCo(new EnumConstantAllCaps())
                .addCoCo(new EnumConstantUnique());
    }
}
