package de.thesis.consumer.backend.domain;

public class InvalidPolicyException extends Exception {
	public InvalidPolicyException (String errorMessage) {
		super(errorMessage);
	}
}
