package exceptions;

public class UseCaseException extends RuntimeException {
	public UseCaseException(String message) {
		super(message);
	}
}
