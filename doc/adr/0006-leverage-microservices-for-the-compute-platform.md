# 6. Leverage microservices for the compute platform

Date: 2025-12-02

## Status

Accepted

## Context

The compute platform is responsible for higher-level tasks such as vision processing, visual SLAM, path planning, OTA updates, and user interface/monitoring. This is an extremely diverse set of responsibilities, and they may change significantly over time as the project evolves. Each task may also be better suited to different languages, frameworks, and design strategies.

One of the key objectives of Benji is to serve as a platform for reasearch and education about robotics, which further drives a desire for flexibility and modularity, especially since the definition of done isn't particularly well-defined.

## Decision

We will leverage a microservices architecture for the compute platform, with each major task implemnented as a separate service. Each service will commmunicate via a messageing fabric to further decouple the services and permit easy modification and refactoring.

## Consequences

Mircoservices are complex, and introduce a bit of overhead in terms of compute and deployment. The compute isn't much of a concern as the compute platform will be relatively powerful. The deployment is mitigated by the use of a monorepo.
