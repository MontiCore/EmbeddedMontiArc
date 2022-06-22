package de.thesis.provider.backend.controller;

import de.thesis.provider.backend.CreateDatasetCommand;
import de.thesis.provider.backend.Metadata;
import de.thesis.provider.backend.service.DatasetService;
import lombok.AllArgsConstructor;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;

import java.io.IOException;
import java.util.Map;
import java.util.UUID;

@RestController
@RequestMapping("/datasets")
@AllArgsConstructor
@CrossOrigin("*")
public class DatasetController {

	private final DatasetService service;

	@PostMapping
	@ResponseStatus(HttpStatus.OK)
	public void createDataset(@RequestBody CreateDatasetCommand command,
							  @Value("${provider.name}") String providerName,
							  @Value("${provider.url}") String providerUrl) throws IOException {
		command.getMetadata().setProvider(providerName);
		command.getMetadata().setLoggingUrl(providerUrl);
		service.offerDataset(command);
	}

	@GetMapping
	@ResponseStatus(HttpStatus.OK)
	public Map<UUID, Metadata> getDatasets() {
		return service.getDatasets();
	}
}
