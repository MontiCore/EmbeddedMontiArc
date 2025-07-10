package de.thesis.provider.backend.controllers;

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

	@PostMapping("/{offerId}")
	@ResponseStatus(HttpStatus.OK)
	public void testPost(@PathVariable UUID offerId) {
		log.info("Dataset {} accessed at {}", offerId, LocalDateTime.now());
	}
}
