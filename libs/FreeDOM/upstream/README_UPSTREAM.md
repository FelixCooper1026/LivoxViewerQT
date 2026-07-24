# FreeDOM upstream snapshot

This directory contains the algorithm core imported from
[LC-Robotics/FreeDOM](https://github.com/LC-Robotics/FreeDOM) at commit
`dcfd5690cafddf121a80bd8ea477807d9656748e` (the upstream `main` branch).

The upstream project is licensed under the MIT License; see `LICENSE`.
LivoxViewerQT keeps the Map, ScanMap, MRMap, RayCaster, DepthImage and FreeDOM
pipeline intact. Local changes remove ROS transport/TF/visualization/PCD-I/O
dependencies and add source-index tracking, structured timings and safe
snapshot access required by the host application.
