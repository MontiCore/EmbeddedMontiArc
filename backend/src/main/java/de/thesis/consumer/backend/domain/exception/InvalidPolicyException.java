package de.thesis.consumer.backend.domain.exception;

public class InvalidPolicyException extends Exception {
	public InvalidPolicyException (String errorMessage) {
		super(errorMessage);
	}
}
