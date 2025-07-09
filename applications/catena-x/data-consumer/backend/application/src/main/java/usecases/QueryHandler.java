package usecases;

import queries.Query;

public interface QueryHandler<Q extends Query, R> {
	R handle(Q query);
}
