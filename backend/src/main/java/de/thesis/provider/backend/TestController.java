package de.thesis.provider.backend;

import lombok.AllArgsConstructor;
import lombok.Value;
import org.springframework.util.LinkedMultiValueMap;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.ArrayList;

@RestController
@AllArgsConstructor
public class TestController {
	private SpringHttpClient client;

	@GetMapping("/test")
	public String test() {
		LinkedMultiValueMap<String, String> map = new LinkedMultiValueMap<>();

		return client.get("/people/1", null, String.class, null);
	}

	@PostMapping("/test")
	public String testPost() {
		return client.post("/search", null, String.class, null);
	}
}
