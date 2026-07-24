# FreeDOM equivalence replay

`FreeDomEquivalenceTool` feeds a CSV point/pose sequence directly to the
ROS-free `freedom_core` and writes order-independent per-frame metrics for the
comparison required by the migration design. The input fields are:

`frame,timestamp_ns,tx,ty,tz,qx,qy,qz,qw,x,y,z`

Build and run:

```powershell
cmake --build build-msvc --config Release --target FreeDomEquivalenceTool
build-msvc\Release\FreeDomEquivalenceTool.exe tests\data\freedom_synthetic_replay.csv migrated.csv
```

Run the same point/pose sequence and Mid-360 parameters through the upstream
ROS1 node, export the metrics listed in section 24.1 of the migration design,
sort static map coordinates before hashing, and compare its CSV with
`migrated.csv`. Coordinate hashes quantize at 1 micrometre so unordered map
iteration order does not affect the result. The bundled synthetic replay is a
deterministic smoke fixture; real PCAP/LVX/rosbag equivalence runs should use
the corresponding FAST_LIO frame-end poses.
