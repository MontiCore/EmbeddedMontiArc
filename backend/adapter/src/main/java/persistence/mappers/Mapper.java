package persistence.mappers;

public interface Mapper<S, T> {
	T mapTo(S s);

	S mapFrom(T t);
}
