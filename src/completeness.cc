// Copyright 2017 Thomas Schöps, Johannes L. Schönberger
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "completeness.h"

#include <pix4d/nanoflannwrap/PointCloudANN.h>
#include <Eigen/Geometry>
#include <tinyply.h>

#include <fstream>

const int kGridCount = 2;
const float kGridShifts[kGridCount][3] = {{0.f, 0.f, 0.f}, {0.5f, 0.5f, 0.5f}};

// Completeness results for one voxel cell.
struct CompletenessCell {
  inline CompletenessCell() : point_count(0) {}

  // Number of scan points within this cell.
  size_t point_count;

  // Number of complete scan points (smaller or equal to point_count), for each
  // tolerance value.
  // Indexed by: [tolerance_index].
  std::vector<size_t> complete_count;
};

void ComputeCompleteness(const MeshLabMeshInfoVector& scan_infos,
                         const std::vector<PointCloud>& scans,
                         const PointCloud& reconstruction,
                         float voxel_size_inv,
                         // Sorted by increasing tolerance.
                         const std::vector<float>& sorted_tolerances,
                         // Indexed by: [tolerance_index]. Range: [0, 1].
                         std::vector<float>* results,
                         // Indexed by: [tolerance_index][scan_point_index].
                         std::vector<std::vector<bool>>* point_is_complete) {
  bool output_point_results = point_is_complete != nullptr;
  size_t tolerances_count = sorted_tolerances.size();
  float maximum_tolerance = sorted_tolerances.back();

  // Compute squared tolerances.
  std::vector<float> sorted_tolerances_squared(tolerances_count);
  for (size_t tolerance_index = 0; tolerance_index < tolerances_count;
       ++tolerance_index) {
    sorted_tolerances_squared[tolerance_index] =
        sorted_tolerances[tolerance_index] * sorted_tolerances[tolerance_index];
  }

  // Merge the scan point clouds into one point cloud in global coordinates.
  PointCloud scan;
  for (size_t scan_index = 0; scan_index < scan_infos.size(); ++scan_index) {
    PointCloud temp_cloud;
    Eigen::Affine3f affine;
    affine.matrix() = scan_infos[scan_index].global_T_mesh;
    std::transform(scans[scan_index].begin(), scans[scan_index].end(), std::back_inserter(scan),
            [&](auto p) -> Eigen::Vector3f { return affine * p;});
  }

  // Prepare point_is_complete, if requested.
  if (output_point_results) {
    point_is_complete->resize(tolerances_count);
    for (size_t tolerance_index = 0; tolerance_index < tolerances_count;
         ++tolerance_index) {
      point_is_complete->at(tolerance_index).resize(scan.size(), false);
    }
  }

  // Special case for empty reconstructions since the KdTree construction
  // crashes for them.
  if (reconstruction.size() == 0) {
    results->clear();
    results->resize(tolerances_count, 0.f);
    return;
  }

  Eigen::Vector3f search_point;
  pix4d::nanoflannwrap::PointCloudANN<3, float> reconstruction_kdtree(reconstruction);

  // Differently shifted voxel grids.
  // Indexed by: [map_index][CalcCellCoordinates(...)].
  std::unordered_map<std::tuple<int, int, int>, CompletenessCell>
      cell_maps[kGridCount];

  const int kNN = 1;
  std::vector<size_t> knn_indices(kNN);
  std::vector<float> knn_squared_dists(kNN);

  const long long int scan_point_size =
      static_cast<long long int>(scan.size());

// Loop over all scan points.
#pragma omp parallel for private(search_point, knn_indices, knn_squared_dists)
  for (long long int scan_point_index = 0; scan_point_index < scan_point_size;
       ++scan_point_index) {
    const auto& scan_point = scan.at(scan_point_index);

    // Find the voxels for this scan point and increase their point count.
    CompletenessCell* cells[kGridCount];
#pragma omp critical
    for (int grid_index = 0; grid_index < kGridCount; ++grid_index) {
      CompletenessCell* cell = &cell_maps[grid_index][CalcCellCoordinates(
          scan_point, voxel_size_inv, kGridShifts[grid_index][0],
          kGridShifts[grid_index][1], kGridShifts[grid_index][2])];
      ++cell->point_count;
      if (cell->complete_count.empty()) {
        cell->complete_count.resize(tolerances_count, 0);
      }
      cells[grid_index] = cell;
    }

    // Find the closest reconstruction point to this scan point, limited to the
    // maximum evaluation tolerance for efficiency.
    search_point = scan_point;
    reconstruction_kdtree.knnSearch(search_point, kNN, knn_indices, knn_squared_dists);
    if (knn_squared_dists.front() <= maximum_tolerance * maximum_tolerance) {
      int smallest_complete_tolerance_index = 0;
#pragma omp critical
      {
        // Since a reconstruction point was found within the search radius, this
        // scan point is complete for the maximum tolerance, at least.
        for (int grid_index = 0; grid_index < kGridCount; ++grid_index) {
          cells[grid_index]->complete_count[tolerances_count - 1] += 1;
        }

        // Next, find the smallest tolerance for which it is still complete.
        for (int tolerance_index = tolerances_count - 2; tolerance_index >= 0;
             --tolerance_index) {
          if (sorted_tolerances_squared[tolerance_index] <
              knn_squared_dists[0]) {
            // The scan point is not completed for the current tolerance index.
            smallest_complete_tolerance_index = tolerance_index + 1;
            break;
          }
          for (int grid_index = 0; grid_index < kGridCount; ++grid_index) {
            cells[grid_index]->complete_count[tolerance_index] += 1;
          }
        }
      }

      // Output point results, if requested.
      if (output_point_results) {
        // The points is incomplete for tolerances smaller than the smallest
        // complete one.
        for (int tolerance_index = 0;
             tolerance_index < smallest_complete_tolerance_index;
             ++tolerance_index) {
          point_is_complete->at(tolerance_index)[scan_point_index] = false;
        }
        // The point is complete for tolerances starting from the smallest
        // complete one.
        for (size_t tolerance_index = smallest_complete_tolerance_index;
             tolerance_index < tolerances_count; ++tolerance_index) {
          point_is_complete->at(tolerance_index)[scan_point_index] = true;
        }
      }
    } else if (output_point_results) {
      // This scan point is incomplete for all tolerances.
      for (size_t tolerance_index = 0; tolerance_index < tolerances_count;
           ++tolerance_index) {
        point_is_complete->at(tolerance_index)[scan_point_index] = false;
      }
    }
  }

  // Average results over all cells and fill the results vector.
  std::vector<double> completeness_sum(tolerances_count, 0.0);
  size_t cell_count = 0;
  for (int grid_index = 0; grid_index < kGridCount; ++grid_index) {
    cell_count += cell_maps[grid_index].size();

    for (auto it = cell_maps[grid_index].cbegin(),
              end = cell_maps[grid_index].cend();
         it != end; ++it) {
      const CompletenessCell& cell = it->second;
      for (size_t tolerance_index = 0; tolerance_index < tolerances_count;
           ++tolerance_index) {
        completeness_sum[tolerance_index] +=
            cell.complete_count[tolerance_index] / (1.0 * cell.point_count);
      }
    }
  }

  results->resize(tolerances_count);
  for (size_t tolerance_index = 0; tolerance_index < tolerances_count;
       ++tolerance_index) {
    results->at(tolerance_index) =
        completeness_sum[tolerance_index] / cell_count;
  }
}

void WriteCompletenessVisualization(
    const std::string& base_path, const MeshLabMeshInfoVector& scan_infos,
    const std::vector<PointCloud>& scans,
    // Sorted by increasing tolerance.
    const std::vector<float>& sorted_tolerances,
    // Indexed by: [tolerance_index][scan_point_index].
    const std::vector<std::vector<bool>>& point_is_complete) {

  // Set visualization point positions (once for all tolerances).
  PointCloud scan;
  for (size_t scan_index = 0; scan_index < scan_infos.size(); ++scan_index) {
    PointCloud temp_cloud;
    Eigen::Affine3f affine;
    affine.matrix() = scan_infos[scan_index].global_T_mesh;
    std::transform(scans[scan_index].begin(), scans[scan_index].end(), std::back_inserter(scan),
            [&](auto p) -> Eigen::Vector3f { return affine * p;});
  }
  std::vector<Eigen::Vector3<uint8_t>> completeness_visualization(scan.size());

  // Loop over all tolerances, set visualization point colors accordingly and
  // save the point clouds.
  // TODO this makes no sense to loop over all tolerances since only the effect of the last tolerance will determine the color of the points. WTH is this code trying to do?
  for (size_t tolerance_index = 0; tolerance_index < sorted_tolerances.size();
       ++tolerance_index) {
    const std::vector<bool>& point_is_complete_for_tolerance =
        point_is_complete[tolerance_index];

    for (size_t scan_point_index = 0;
         scan_point_index < completeness_visualization.size();
         ++scan_point_index) {
      auto& point =
          completeness_visualization.at(scan_point_index);
      if (point_is_complete_for_tolerance[scan_point_index]) {
        // Green: complete points.
        point.x() = 0;
        point.y() = 255;
        point.z() = 0;
      } else {
        // Red: incomplete points.
        point.x() = 255;
        point.y() = 0;
        point.z() = 0;
      }
    }

    std::ostringstream file_path;
    file_path << base_path << ".tolerance_"
              << sorted_tolerances[tolerance_index] << ".ply";

    tinyply::PlyFile ply;
    ply.add_properties_to_element("vertex", {"x", "y", "z"},
            tinyply::Type::FLOAT32, scan.size(),
            reinterpret_cast<const uint8_t*>(scan.data()), tinyply::Type::INVALID, 0);

    ply.add_properties_to_element("vertex", {"diffuse_red", "diffuse_green", "diffuse_blue"},
            tinyply::Type::UINT8, completeness_visualization.size(),
            reinterpret_cast<const uint8_t*>(completeness_visualization.data()), tinyply::Type::INVALID, 0);

    std::ofstream out;
    out.exceptions(std::ios_base::badbit | std::ios_base::failbit);

    out.open(file_path.str(), std::ios_base::binary);

    ply.write(out, true);
  }
}
