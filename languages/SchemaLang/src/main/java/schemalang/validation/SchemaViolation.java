package schemalang.validation;

import java.util.Objects;

public class SchemaViolation extends Violation {

    protected String schemaName;

    private SchemaViolation(String errorCode, String errorMessage) {
        super(errorCode, errorMessage);
    }

    public String getSchemaName() {
        return schemaName;
    }

    public void setSchemaName(String schemaName) {
        this.schemaName = schemaName;
    }

    public static SchemaViolation create(String errorCode, String errorMessage, String schemaName) {
        SchemaViolation violation = new SchemaViolation(errorCode, errorMessage);
        violation.setSchemaName(schemaName);
        return violation;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SchemaViolation that = (SchemaViolation) o;
        return Objects.equals(errorCode, that.errorCode) && Objects.equals(errorMessage, that.errorMessage) && Objects.equals(schemaName, that.schemaName);
    }

    @Override
    public int hashCode() {
        return Objects.hash(errorCode, errorMessage, schemaName);
    }

    @Override
    public String toString() {
        return errorCode.concat(errorMessage);
    }
}