package de.thesis.provider.backend.controller;

import com.opencsv.bean.CsvToBean;
import com.opencsv.bean.CsvToBeanBuilder;
import de.thesis.provider.backend.csv.DataRow;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.util.List;

@RestController
@RequestMapping("/fileupload")
@CrossOrigin("*")
public class FileController {

	@PostMapping
	@ResponseStatus(HttpStatus.OK)
	public List<DataRow> getDatarows(@RequestParam("file") MultipartFile file) throws IOException {
		Reader reader = new BufferedReader(new
				InputStreamReader(file.getInputStream(), StandardCharsets.UTF_8));
		CsvToBean<DataRow> csvReader = new CsvToBeanBuilder<DataRow>(reader)
				.withType(DataRow.class)
				.withSeparator(';')
				.withIgnoreLeadingWhiteSpace(true)
				.withIgnoreEmptyLine(true)
				.build();

		return csvReader.parse();
	}
}
