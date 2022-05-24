package de.thesis.provider.backend.controller;

import de.thesis.provider.backend.Dataset;
import de.thesis.provider.backend.DatasetCreateCommand;
import de.thesis.provider.backend.service.DatasetService;
import freemarker.template.TemplateException;
import lombok.AllArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;

import java.io.IOException;
import java.util.List;

@RestController
@RequestMapping("/datasets")
@AllArgsConstructor
@CrossOrigin(origins = "*")
public class DatasetController {

	private final DatasetService service;

	@PostMapping
	@ResponseStatus(HttpStatus.OK)
	public void createDataset(@RequestBody DatasetCreateCommand command) throws IOException, TemplateException {
		service.offerDataset(command);
	}

	@GetMapping
	@ResponseStatus(HttpStatus.OK)
	public List<Dataset> getDatasets() {
		return service.getDatasets();
	}
}
