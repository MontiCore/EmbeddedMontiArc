package conflangliterals;

import com.google.common.collect.Lists;
import conflangliterals._ast.ASTComponentLiteral;
import conflangliterals._ast.ASTListLiteral;
import conflangliterals._ast.ASTTypelessLiteral;
import de.monticore.mcliterals._ast.*;
import de.se_rwth.commons.logging.Log;

import java.util.List;

public class LiteralHelpers {

    private LiteralHelpers() {
        // hidden constructor
    }

    public static Object literalValue(ASTSignedLiteral signedLiteral) {
        if (signedLiteral instanceof ASTSignedIntLiteral) {
            ASTSignedIntLiteral signedIntLiteral = (ASTSignedIntLiteral) signedLiteral;
            return signedIntLiteral.getValue();

        } else if (signedLiteral instanceof ASTSignedDoubleLiteral) {
            ASTSignedDoubleLiteral signedDoubleLiteral = (ASTSignedDoubleLiteral) signedLiteral;
            Double value = signedDoubleLiteral.getValue();
            return value.toString();

        } else if (signedLiteral instanceof ASTBooleanLiteral) {
            ASTBooleanLiteral booleanLiteral = (ASTBooleanLiteral) signedLiteral;
            if (booleanLiteral.getValue()) {
                return "True";
            }
            return "False";

        } else if (signedLiteral instanceof ASTStringLiteral) {
            ASTStringLiteral stringLiteral = (ASTStringLiteral) signedLiteral;
            return stringLiteral.getValue();

        } else if (signedLiteral instanceof ASTTypelessLiteral) {
            ASTTypelessLiteral typelessLiteral = (ASTTypelessLiteral) signedLiteral;
            return "'".concat(typelessLiteral.getValue().concat("'"));

        } else if (signedLiteral instanceof ASTListLiteral) {
            ASTListLiteral listLiteral = (ASTListLiteral) signedLiteral;
            List<Object> vectorEntries = Lists.newArrayList();
            for (ASTSignedLiteral literal : listLiteral.getSignedLiteralList()) {
                vectorEntries.add(literalValue(literal));
            }
            return vectorEntries;
        }
        return signedLiteral.toString();
    }

    public static String literalToString(ASTSignedLiteral initialValue) {
        if (initialValue instanceof ASTListLiteral) {
            ASTListLiteral listLiteral = (ASTListLiteral) initialValue;
            return listLiteral.toString();

        } else if (initialValue instanceof ASTTypelessLiteral) {
            ASTTypelessLiteral typelessLiteral = (ASTTypelessLiteral) initialValue;
            return typelessLiteral.toString();

        } else if (initialValue instanceof ASTComponentLiteral) {
            ASTComponentLiteral componentLiteral = (ASTComponentLiteral) initialValue;
            return componentLiteral.toString();

        } else if (initialValue instanceof ASTStringLiteral) {
            ASTStringLiteral stringLiteral = (ASTStringLiteral) initialValue;
            return "\"".concat(stringLiteral.getValue()).concat("\"");

        } else if (initialValue instanceof ASTSignedIntLiteral) {
            ASTSignedIntLiteral signedIntLiteral = (ASTSignedIntLiteral) initialValue;
            return String.valueOf(signedIntLiteral.getValue());

        } else if (initialValue instanceof ASTSignedDoubleLiteral) {
            ASTSignedDoubleLiteral signedDoubleLiteral = (ASTSignedDoubleLiteral) initialValue;
            return String.valueOf(signedDoubleLiteral.getValue());

        } else if (initialValue instanceof ASTBooleanLiteral) {
            ASTBooleanLiteral booleanLiteral = (ASTBooleanLiteral) initialValue;
            return String.valueOf(booleanLiteral.getValue());
        }
        Log.warn(String.format("Type '%s' is not supported.", initialValue));
        return null;
    }
}