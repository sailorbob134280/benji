# 7. Use NATS for the compute message bus

Date: 2025-12-02

## Status

Accepted

## Context

With the decision to leverage a microservices architecture for the compute platform, we need to select a messaging fabric for inter-service communication. The messaging fabric should be light enough for an embedded platform and support C++, Go, and Python clients, as those are the primary languages that will be used for the compute platform services. It should also support a variety of interaction models, including publish/subscribe and request/response.

## Decision

We will use NATS as the messaging fabric for the compute platform. NATS is both easy to deploy (as it's written in Go) and very lightweight, making it a good fit for our use case. It also has support for a wide variety of interaction models, and client libraries for the languages we're interested in. The API is also very simple.

## Consequences

NATS isn't traditionally aimed at embedded systems, and it lacks some more advanced features commonly found in something like MQTT (e.g. QoS). It is also known to have slightly higher latency than MQTT, and substantially higher latency than a brokerless system like ZeroMQ. For this use case, latency isn't much of a concern, and we value simplicity and ease of use much more highly.
