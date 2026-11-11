# Local Ogre1 pixel-centre candidate (Ignition Rendering 6.6.4)

Official source: https://github.com/gazebosim/gz-rendering/tree/ignition-rendering6_6.6.4

Only upstream `ogre/src/OgreGpuRays.cc` is changed; see `pixel_centres.patch`.
Canvas POINTS are shifted from output-pixel edges to centres. Ray angles, UVs,
range values, PointCloud conversion, sensors, robot geometry and system packages
are not altered. This remains a candidate until raw-range A/B validation passes.

The custom CMake builds only Ogre1 sources and links installed version-6 core,
common and Ogre1.9 libraries. No core/Ogre2 is rebuilt or installed. Output stays
in `overlay/lib`. The installed engine contains a direct local CreateMesh call,
so interposing one translation unit with LD_PRELOAD would not reliably patch it.

Build (explicit user authorization exists in the parent task):

```bash
cmake -S artifacts/simulation_integration/rendering_patch -B artifacts/simulation_integration/rendering_patch/build -DCMAKE_BUILD_TYPE=Release
cmake --build artifacts/simulation_integration/rendering_patch/build --parallel 4
```

The first attempt failed because the custom build omitted the existing Ogre GL
header directory; adding `/usr/include/OGRE/RenderSystems/GL` fixed this build
configuration omission. `build_attempt1.log` retains that failure.

Reliable explicit selection for a fresh test world:

```xml
<render_engine>/home/liu/ds_nav/artifacts/simulation_integration/rendering_patch/overlay/lib/libignition-rendering6-ogre.so.6.6.4</render_engine>
```

The engine remains Ogre1; the absolute library name prevents the manager from
translating the default name into a system-first search. `path_probe.cc/log`
prove the installed SystemPaths behavior: system paths added by the manager
precede the lazily appended `IGN_RENDERING_PLUGIN_PATH` directory. Setting that
environment variable or LD_LIBRARY_PATH alone does not reliably select this
backend. An initial parent test loaded the system library, so its unchanged
ranges are not evidence against the patch.

Check `/proc/<Gazebo-server-PID>/maps` to prove the local backend loaded;
otherwise the A/B result is invalid. Keep this override in a test world copy or
an explicit launch option; do not change global shell profiles or system libs.
No simulation has been started or stopped by this subtask.

`version.json` records source/package hashes; `canvas_static_test.json` checks
all row/column centres at 360x16, 720x1 and 7x3. Runtime acceptance must inspect
first/last row and horizontal columns as well as ground-plane middle rings.


## Installed runtime (2026-11-01)

The tested 6.6.4 shared library is now installed alongside this file by huananhu_sim_bringup. `simulation.launch.py` resolves `render_engine_library:=auto` to this exact library and checks SHA256.json before starting Gazebo. No system library is overwritten. pixel_centres.patch, version.json and LICENSE accompany the binary. This binary was tested on this workspace host; it is not a portable substitute for verifying ABI on a different ROS/Gazebo installation.
