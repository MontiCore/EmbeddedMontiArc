package utilities;

import de.fraunhofer.iese.mydata.policy.Policy;
import de.fraunhofer.iese.mydata.timer.Timer;
import org.springframework.core.io.DefaultResourceLoader;
import org.springframework.core.io.Resource;
import org.springframework.core.io.ResourceLoader;
import org.springframework.util.FileCopyUtils;

import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.UncheckedIOException;

import static java.nio.charset.StandardCharsets.UTF_8;

public class MydataResourceReader {

	private static final String TIMER_PATH = "classpath:mydata/timers/";
	private static final String POLICY_PATH = "classpath:mydata/policies/";

	public static Policy readPolicyFromFile(String filename) {
		String path = POLICY_PATH + filename;
		ResourceLoader resourceLoader = new DefaultResourceLoader();
		Resource resource = resourceLoader.getResource(path);

		return new Policy(resourceToString(resource));
	}

	public static Timer readTimerFromFile(String filename) {
		String path = TIMER_PATH + filename;
		ResourceLoader resourceLoader = new DefaultResourceLoader();
		Resource resource = resourceLoader.getResource(path);

		return new Timer(resourceToString(resource));
	}

	public static String resourceToString(Resource resource) {
		try (Reader reader = new InputStreamReader(resource.getInputStream(), UTF_8)) {
			return FileCopyUtils.copyToString(reader);
		} catch (IOException e) {
			throw new UncheckedIOException(e);
		}
	}
}
