package port;

public interface PolicyEnforcementPort<T> {
	T enforce(T object);
}
