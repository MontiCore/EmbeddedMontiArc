package schemalang.exception;

public class SchemaLangTechnicalException extends RuntimeException {

    public SchemaLangTechnicalException() {
    }

    public SchemaLangTechnicalException(String message) {
        super(message);
    }

    public SchemaLangTechnicalException(String message, Throwable cause) {
        super(message, cause);
    }

    public SchemaLangTechnicalException(Throwable cause) {
        super(cause);
    }

    public SchemaLangTechnicalException(String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
        super(message, cause, enableSuppression, writableStackTrace);
    }
}
