package schemalang.validation.exception;

public class SchemaLangException extends Exception {

    public SchemaLangException() {
    }

    public SchemaLangException(String message) {
        super(message);
    }

    public SchemaLangException(String message, Throwable cause) {
        super(message, cause);
    }

    public SchemaLangException(Throwable cause) {
        super(cause);
    }

    public SchemaLangException(String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
        super(message, cause, enableSuppression, writableStackTrace);
    }
}
