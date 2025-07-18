package schemalang.validation.exception;

public class SchemaLangValidationException extends SchemaLangException {

    public SchemaLangValidationException() {
    }

    public SchemaLangValidationException(String message) {
        super(message);
    }

    public SchemaLangValidationException(String message, Throwable cause) {
        super(message, cause);
    }

    public SchemaLangValidationException(Throwable cause) {
        super(cause);
    }

    public SchemaLangValidationException(String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
        super(message, cause, enableSuppression, writableStackTrace);
    }
}
