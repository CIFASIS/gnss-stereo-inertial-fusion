# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.0] - 2023-07-13

### Added
- When a Keyframe is associated to a GNSS measurement, the SE(3) pose is propagated to the GNSS measurement timestamp. This differs from the paper, originally it was assumed that the Keyframe and the GNSS measurement have the same timestamp (actually they were temporally close but the offset was not zero)

### Fixed


### Changed


### Removed

