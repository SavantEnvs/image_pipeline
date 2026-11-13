# syntax=docker/dockerfile:1.7
FROM ghcr.io/savantenvs/base:20261023

ARG COMMIT
ARG REPO_URL
ARG SOURCE_DATE_EPOCH
ARG SANITIZER_FLAGS
ARG COVERAGE_FLAGS=
ARG MAYHEM_JOBS=

# OpenCV for the fuzzed library code. ROS 2 is not in Debian, so the two ROS interface packages the
# code needs headers from are pinned by commit under /opt/toolchains/deps (SPEC §6.2 item 19).
USER root
RUN [ -n "$SOURCE_DATE_EPOCH" ] && repro-sources-list.sh || true ; \
    apt-get update && apt-get install -yq --no-install-recommends libopencv-dev pkg-config git ca-certificates \
 && rm -rf /var/lib/apt/lists/*

ARG COMMON_INTERFACES_SHA=05a5c4e6a42f89b71745320c34c467cb391a94b9
ARG VISION_OPENCV_SHA=a56ad9a332218096da5916534a367c8c54a49e16
RUN set -e; \
    for spec in "ros2/common_interfaces:${COMMON_INTERFACES_SHA}:common_interfaces" \
                "ros-perception/vision_opencv:${VISION_OPENCV_SHA}:vision_opencv"; do \
      repo=${spec%%:*}; rest=${spec#*:}; sha=${rest%%:*}; name=${rest#*:}; \
      d=/opt/toolchains/deps/${name}-${sha} && mkdir -p "$d/src" && cd "$d/src" \
      && git init -q && git fetch -q --depth 1 https://github.com/${repo}.git "$sha" \
      && git checkout -q FETCH_HEAD && [ "$(git rev-parse HEAD)" = "$sha" ] \
      && rm -rf .git && chmod -R a+rX "$d"; \
    done

USER mayhem
WORKDIR /mayhem
COPY --chown=2000:2000 . /mayhem

RUN if [ -x /opt/mayhem/ccache/prefill ]; then /opt/mayhem/ccache/prefill bash mayhem/build.sh; \
    else bash mayhem/build.sh; fi

RUN if [ -f mayhem/test.sh ]; then bash mayhem/test.sh; else echo "no mayhem/test.sh — repo ships no functional tests"; fi

LABEL org.opencontainers.image.source="${REPO_URL}" \
      org.opencontainers.image.revision="${COMMIT}" \
      rlenv.tier="commit" \
      rlenv.source_date_epoch="${SOURCE_DATE_EPOCH}" \
      rlenv.user="mayhem"

USER mayhem
WORKDIR /mayhem
