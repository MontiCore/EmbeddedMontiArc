package persistence.mappers;

public interface Mapper<S, T> {
	T mapToPersistenceEntity(S s);

	S mapToDomainEntity(T t);
}
