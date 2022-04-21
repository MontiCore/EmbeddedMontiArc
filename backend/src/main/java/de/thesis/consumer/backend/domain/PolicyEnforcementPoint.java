package de.thesis.consumer.backend.domain;

public interface PolicyEnforcementPoint<T> {
	T enforce(T object) throws InhibitionException;
}
