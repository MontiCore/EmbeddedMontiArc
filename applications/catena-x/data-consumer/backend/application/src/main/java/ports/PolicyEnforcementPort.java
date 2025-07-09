package ports;

public interface PolicyEnforcementPort<T> {
	T enforcePolicy(T type);
}
