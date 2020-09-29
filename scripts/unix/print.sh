#!/bin/bash


# Use colors to identify scripts messages better:
CYAN='\033[1;36m'
NC='\033[0m' # No Color


function print {
    echo -e "${CYAN}[SCRIPT] $@${NC}"
}
