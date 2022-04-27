package de.thesis.consumer.backend.presentation.controller;

import de.thesis.consumer.backend.domain.exception.DatasetNotFoundException;
import de.thesis.consumer.backend.domain.model.Dataset;
import de.thesis.consumer.backend.domain.service.DatasetService;
import lombok.AllArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.server.ResponseStatusException;

import java.util.List;
import java.util.UUID;

@RestController
@RequestMapping("/datasets")
@AllArgsConstructor
@CrossOrigin(origins = "*")
public class DatasetController {

	private DatasetService service;

	@GetMapping
	public List<Dataset> getAllDatasets() {
		return service.getAllDatasets();
	}

	@GetMapping("/{id}")
	public Dataset getDataset(@PathVariable UUID id) {
		try {
			return service.getDataset(id);
		} catch (DatasetNotFoundException e) {
			throw new ResponseStatusException(HttpStatus.NOT_FOUND, String.format("Dataset with ID %s not found", id.toString()));
		} catch (RuntimeException e) {
			throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "Policy violation");
		}
	}
}
