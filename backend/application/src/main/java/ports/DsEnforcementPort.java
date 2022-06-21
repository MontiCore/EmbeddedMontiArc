package ports;

public interface DsEnforcementPort<T> {
	T enforce(T type);
}
