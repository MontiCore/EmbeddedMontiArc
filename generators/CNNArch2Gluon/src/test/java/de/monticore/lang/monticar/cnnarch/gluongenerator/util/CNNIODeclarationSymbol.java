package de.monticore.lang.monticar.cnnarch.gluongenerator.util;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;

public class CNNIODeclarationSymbol extends IODeclarationSymbol {

    public CNNIODeclarationSymbol(String name) {
        super(name);
    }

    @Override
    public void setType(ArchTypeSymbol type) {
        super.setType(type);
    }
}
