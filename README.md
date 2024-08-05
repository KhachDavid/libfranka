# libfranka: C++ library for Franka Robotics research robots

# Northwestern MSR Changes
1. This library is now meant to be used with the Franka Emika Panda Robot, that is officially only compatible with v0.9.0 of libfranka
   - It is modified from v0.13.3 of libfranka to be compatible with the Panda
   - Velocity, joint, and rate limits are set to be that of the Panda
2. The Panda and FR3 robots are very similar but have slightly different capabilities: use at your own risk!

[![Build Status][travis-status]][travis]
[![codecov][codecov-status]][codecov]

With this library, you can control research versions of Franka Robotics robots. See the [Franka Control Interface (FCI) documentation][fci-docs] for more information about what `libfranka` can do and how to set it up. The [generated API documentation][api-docs] also gives an overview of its capabilities.

## License

`libfranka` is licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[api-docs]: https://frankaemika.github.io/libfranka
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/libfranka.svg?branch=master
[travis]: https://travis-ci.org/frankaemika/libfranka
[codecov-status]: https://codecov.io/gh/frankaemika/libfranka/branch/master/graph/badge.svg
[codecov]: https://codecov.io/gh/frankaemika/libfranka
