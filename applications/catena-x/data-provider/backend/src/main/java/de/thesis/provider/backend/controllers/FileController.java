package de.thesis.provider.backend.controllers;

import de.thesis.provider.backend.csv.CsvReader;
import de.thesis.provider.backend.csv.DataRow;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.util.List;

@RestController
@RequestMapping("/fileupload")
@CrossOrigin("*")
public class FileController {

	@PostMapping
	@ResponseStatus(HttpStatus.OK)
	public List<DataRow> getDatarows(@RequestParam("file") MultipartFile file) throws IOException {
		CsvReader reader = new CsvReader();

		return reader.getCsvData(file);
	}
}
