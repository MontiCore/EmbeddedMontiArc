package entity;

import lombok.Data;

import java.time.LocalDate;
import java.time.LocalTime;

@Data
public class Policy {
	private Long id;
	private LocalTime startTime;
	private LocalTime endTime;
	private LocalDate expiresOn;
	private Integer maxUsages;
	private boolean localLogging;
	private boolean remoteLogging;
}
