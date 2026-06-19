# Third-Party Headers

This release vendors only the header subsets needed by `PointCloudProcessor`:

- `Sophus/sophus`: Lie group headers from Sophus.
- `json/include`: nlohmann/json headers.

The original projects and licenses are:

- Sophus: https://github.com/strasdat/Sophus
- nlohmann/json: https://github.com/nlohmann/json

If you prefer submodules, remove these vendored folders and add the upstream repositories at the same paths.
