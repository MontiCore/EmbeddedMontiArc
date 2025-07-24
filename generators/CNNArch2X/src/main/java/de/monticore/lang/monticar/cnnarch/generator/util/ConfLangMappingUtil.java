package de.monticore.lang.monticar.cnnarch.generator.util;

import conflangliterals._ast.ASTTypelessLiteral;
import de.monticore.mcliterals._ast.ASTSignedDoubleLiteral;
import de.monticore.mcliterals._ast.ASTSignedIntLiteral;
import de.monticore.mcliterals._ast.ASTSignedLiteral;

/***
 * used in FreeMarker templates
 */
@SuppressWarnings("unused")
public class ConfLangMappingUtil {

    public static String mapConfigurationValueToPythonValue(ASTSignedLiteral configurationValue) {
        if (configurationValue instanceof ASTSignedIntLiteral)
            return ((ASTSignedIntLiteral) configurationValue).getSource();
        if (configurationValue instanceof ASTSignedDoubleLiteral)
            return ((ASTSignedDoubleLiteral) configurationValue).getSource();
        if (configurationValue instanceof ASTTypelessLiteral)
            return "'" + ((ASTTypelessLiteral) configurationValue).getValue() + "'";
        throw new IllegalArgumentException("Provided configuration value is not supported");
    }
}
