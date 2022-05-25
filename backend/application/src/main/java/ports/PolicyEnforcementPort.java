package ports;

public interface PolicyEnforcementPort<T> {
	T enforce(T object);
}
