package de.thesis.consumer.backend.persistence.mapper;

public interface Mapper<S, T> {
	T mapTo(S s);

	S mapFrom(T t);
}
