package ports;

public interface HttpClientPort {
	<T> T post(String path,
			   Object requestBody,
			   Class<T> responseType);
}

