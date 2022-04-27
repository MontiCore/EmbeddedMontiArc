package de.thesis.provider.backend;

import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDateTime;
import java.util.UUID;

@RestController
@RequestMapping("/logging")
@AllArgsConstructor
@Slf4j
public class LoggingController {

	@PostMapping("/{datasetId}")
	@ResponseStatus(HttpStatus.OK)
	public void testPost(@PathVariable UUID datasetId) {
		log.info("Dataset {} accessed at {}", datasetId, LocalDateTime.now());
	}
}
