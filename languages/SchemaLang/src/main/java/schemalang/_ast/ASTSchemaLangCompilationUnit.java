package schemalang._ast;

import com.google.common.base.Joiner;
import de.monticore.mcbasictypes1._ast.ASTImportStatement;

import java.util.List;

public class ASTSchemaLangCompilationUnit extends ASTSchemaLangCompilationUnitTOP {

    public ASTSchemaLangCompilationUnit() {
    }

    public ASTSchemaLangCompilationUnit(List<String> packages, List<ASTImportStatement> importStatements, ASTSchemaDefinition schemaLangDefinition) {
        super(packages, importStatements, schemaLangDefinition);
    }

    public String getPackageName() {

        List<String> packageList = getPackageList();
        if (packageList == null || packageList.isEmpty()) {
            return null;
        }
        return Joiner.on(".").join(packageList);
    }
}