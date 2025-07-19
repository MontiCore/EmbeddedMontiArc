#!/bin/bash

env=$(ls | grep environment)
cd ./*environment
java -cp target/$env-*.jar de.gdl.rl.environment.games.doppelkopf.DoppelkopfEnvironment --training
