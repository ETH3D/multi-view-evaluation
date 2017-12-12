# ETH3D Multi-View Evaluation Program #

This tool is used for evaluating multi-view reconstruction methods in the [ETH3D benchmark](https://www.eth3d.net/).

Example usage:

```
ETH3DMultiViewEvaluation --reconstruction_ply_path reconstruction.ply \
                         --ground_truth_mlp_path scan_alignment.mlp \
                         --tolerances 0.01,0.02,0.05,0.1,0.2,0.5
```

Description of required program arguments:

* `--reconstruction_ply_path`: Path to the reconstructed point cloud (as PLY).
* `--ground_truth_mlp_path`: Path to the MeshLab project file which defines the poses of the ground truth laser scan files.
* `--tolerances`: Comma-separated list of tolerance values to evaluate with.

Description of optional program arguments:

* `--voxel_size` (default 0.01): Size of voxels for normalising scores per volume.
* `--beam_start_radius_meters` (default 0.5 * 0.00225): Size of beam at the laser scanner origin for free-space modeling.
* `--beam_divergence_halfangle_deg` (default 0.011): Beam divergence for free-space modeling.
* `--completeness_cloud_output_path` (default ""): If set to a path, completeness visualizations for each tolerance value are written to `<path>.tolerance_<tolerance>.ply`.
* `--accuracy_cloud_output_path` (default ""): If set to a path, accuracy visualizations for each tolerance value are written to `<path>.tolerance_<tolerance>.ply`.
