package de.thesis.consumer.backend.domain;

public class PolicyServiceMock implements PolicyService{
	@Override
	public boolean isValid(String policy) {
		return true;
	}
}
