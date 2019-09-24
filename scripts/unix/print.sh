#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#
# The license generally applicable for this project
# can be found under https://github.com/MontiCore/monticore.
#

# (c) https://github.com/MontiCore/monticore  
#

# Use colors to identify scripts messages better:
CYAN='\033[1;36m'
NC='\033[0m' # No Color


function print {
    echo -e "${CYAN}[SCRIPT] $@${NC}"
}
