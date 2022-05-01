package de.thesis.provider.backend.csv;

import com.opencsv.bean.AbstractBeanField;
import com.opencsv.exceptions.CsvConstraintViolationException;
import com.opencsv.exceptions.CsvDataTypeMismatchException;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class LocalDateConverter extends AbstractBeanField<LocalDateTime, String> {

	@Override
	protected LocalDateTime convert(String rawValue) throws CsvDataTypeMismatchException, CsvConstraintViolationException {
		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd'T'HH:mm:ss'Z'");
		return LocalDateTime.parse(rawValue, formatter);
	}
}
