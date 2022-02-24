#!/bin/bash

#  Copyright (C) 2018-2022 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

set -exo pipefail

dir=~
while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -d|--develop)
                  BRANCH=develop
                  shift
            ;;
            -r|--root)
                  dir=$2
                  shift
                  shift
            ;;
      esac
done

cd ${dir}/autoware.auto

if [[ "$BRANCH" = "develop" ]]; then
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-msgs.git --branch ${BRANCH}
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-utils.git --branch ${BRANCH}
      git clone --depth=1 https://github.com/usdot-fhwa-stol/autoware.auto.git --branch develop
else
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-msgs.git --branch develop
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-utils.git --branch develop
      git clone --depth=1 https://github.com/usdot-fhwa-stol/autoware.auto.git --branch develop
fi

# Required to build pacmod_msgs
git clone https://github.com/astuff/astuff_sensor_msgs.git ${dir}/src/astuff_sensor_msgs --branch 3.0.1
