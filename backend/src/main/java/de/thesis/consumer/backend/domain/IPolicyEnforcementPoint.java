package de.thesis.consumer.backend.domain;

public interface IPolicyEnforcementPoint<T> {
	T enforce(T object);
}
