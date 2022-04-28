package de.thesis.provider.backend.policycreator;

import lombok.Value;

import java.time.LocalTime;

@Value
public class TimeRule {
	LocalTime before;
	LocalTime after;
}
