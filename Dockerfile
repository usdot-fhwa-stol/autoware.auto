#NOTE: Docker image is not really used in carma-platform
# This is just to catch build issues in the autoware.auto repository
# The actual installation of the autoware.auto packages should be done in autoware.ai
ARG DOCKER_ORG=usdotfhwastoldev
ARG DOCKER_TAG=develop-humble
FROM ${DOCKER_ORG}/carma-base:${DOCKER_TAG} as base_image

FROM base_image as build
ARG GIT_BRANCH=develop

COPY --chown=carma . /home/carma/autoware.auto
RUN chmod -R 775 /home/carma/autoware.auto/docker/checkout.bash
RUN chmod -R 775 /home/carma/autoware.auto/docker/install.sh

# NOTE: CARMA doesn't use the debian version of lanelet2, but they are required for autoware.auto
# to build. So installing here so that the autoware.auto build doesn't fail.
RUN sudo apt update && sudo apt install ros-humble-lanelet2-core \
ros-humble-lanelet2-io ros-humble-lanelet2-projection ros-humble-lanelet2-routing -y

RUN /home/carma/autoware.auto/docker/checkout.bash -b ${GIT_BRANCH}
RUN /home/carma/autoware.auto/docker/install.sh

FROM base_image

ARG BUILD_DATE="NULL"
ARG VCS_REF="NULL"
ARG VERSION="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="autoware.auto"
LABEL org.label-schema.description="Binary applications and libraries from autoware.ai for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/autoware.auto"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --from=build --chown=carma /home/carma/install /opt/carma/install
COPY --from=build --chown=carma /opt/ros /opt/ros
