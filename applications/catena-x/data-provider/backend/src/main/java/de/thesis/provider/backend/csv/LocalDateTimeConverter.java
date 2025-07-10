package de.thesis.provider.backend.csv;

import com.opencsv.bean.AbstractBeanField;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeFormatterBuilder;
import java.time.temporal.ChronoField;

public class LocalDateTimeConverter extends AbstractBeanField<LocalDateTime, String> {

	@Override
	protected LocalDateTime convert(String rawValue) {
		DateTimeFormatter formatter = new DateTimeFormatterBuilder()
				.appendPattern("yyyy-MM-dd'T'HH:mm:ss")
				.appendFraction(ChronoField.MILLI_OF_SECOND, 0, 8, true) // min 2 max 3
				.appendPattern("'Z'")
				.toFormatter();
		return LocalDateTime.parse(rawValue, formatter);
	}
}
