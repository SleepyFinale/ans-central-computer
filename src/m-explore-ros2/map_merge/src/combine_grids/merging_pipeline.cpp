/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <combine_grids/grid_compositor.h>
#include <combine_grids/grid_warper.h>
#include <combine_grids/merging_pipeline.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>

#include "estimation_internal.h"

namespace combine_grids
{
bool MergingPipeline::estimateTransforms(
  rclcpp::Logger logger,
  FeatureType feature_type,
  double confidence,
  double min_inlier_ratio,
  int min_inliers)
{
  std::vector<cv::detail::ImageFeatures> image_features;
  std::vector<cv::detail::MatchesInfo> pairwise_matches;
  std::vector<cv::detail::CameraParams> transforms;
  std::vector<int> good_indices;
  // TODO investigate value translation effect on features
  auto finder = internal::chooseFeatureFinder(feature_type);
  cv::Ptr<cv::detail::FeaturesMatcher> matcher =
    cv::makePtr<cv::detail::AffineBestOf2NearestMatcher>();
  cv::Ptr<cv::detail::Estimator> estimator =
    cv::makePtr<cv::detail::AffineBasedEstimator>();
  cv::Ptr<cv::detail::BundleAdjusterBase> adjuster =
    cv::makePtr<cv::detail::BundleAdjusterAffinePartial>();

  if (images_.empty()) {
    return true;
  }

  /* find features in images */
  RCLCPP_DEBUG(logger, "[estimateTransforms] computing features");
  image_features.reserve(images_.size());
  for (const cv::Mat & image : images_) {
    image_features.emplace_back();
    if (!image.empty()) {
#if CV_VERSION_MAJOR >= 4
      cv::detail::computeImageFeatures(finder, image, image_features.back());
#else
      (*finder)(image, image_features.back());
#endif
    }
  }
  finder = {};

  // AffineBestOf2NearestMatcher runs a 2-neighbor search. OpenCV asserts
  // when a map has fewer than 2 keypoints, which is normal on a fresh scan.
  size_t sparse_maps = 0;
  for (const auto & features : image_features) {
    if (features.keypoints.size() < 2) {
      ++sparse_maps;
    }
  }
  if (sparse_maps > 0) {
    RCLCPP_INFO(
      logger,
      "[estimateTransforms] %zu map(s) have fewer than 2 features; "
      "keeping the last accepted alignment if there is one",
      sparse_maps);
    placeWithoutMatch();
    return true;
  }

  /* find corespondent features */
  RCLCPP_DEBUG(logger, "[estimateTransforms] pairwise matching features");
  try {
    (*matcher)(image_features, pairwise_matches);
  } catch (const cv::Exception & e) {
    RCLCPP_WARN(
      logger,
      "[estimateTransforms] Feature matching failed (%s); "
      "keeping the last accepted alignment if there is one",
      e.what());
    placeWithoutMatch();
    return true;
  }
  matcher = {};

#ifndef NDEBUG
  internal::writeDebugMatchingInfo(images_, image_features, pairwise_matches);
#endif

  // Guard against very weak matches even if OpenCV reports some connectivity.
  // Require a minimum number of inliers and inlier ratio from the best pair.
  int best_inliers = 0;
  int best_matches = 0;
  double best_confidence = 0.0;
  for (const auto & mi : pairwise_matches) {
    if (mi.matches.empty()) {
      continue;
    }
    if (mi.num_inliers > best_inliers) {
      best_inliers = mi.num_inliers;
      best_matches = static_cast<int>(mi.matches.size());
      best_confidence = mi.confidence;
    }
  }

  double best_ratio = (best_matches > 0) ?
    (static_cast<double>(best_inliers) /
    static_cast<double>(best_matches)) :
    0.0;

  const int kMinInliers = std::max(3, min_inliers);
  if (best_inliers > 0 &&
    (best_inliers < kMinInliers || best_ratio < min_inlier_ratio))
  {
    RCLCPP_INFO(
      logger,
      "[estimateTransforms] Best feature match too weak "
      "(inliers=%d, matches=%d, ratio=%.3f, confidence=%.3f); "
      "keeping the last accepted alignment if there is one",
      best_inliers, best_matches, best_ratio, best_confidence);

    placeWithoutMatch();

    RCLCPP_DEBUG(
      logger,
      "[estimateTransforms] No new alignment due to weak feature matches");
    return true;
  }

  /* use only matches that has enough confidence. leave out matches that are not
   * connected (small components) */
  good_indices = cv::detail::leaveBiggestComponent(
    image_features, pairwise_matches, static_cast<float>(confidence));

  // no match found between any pair of maps.
  if (good_indices.size() <= 1) {
    placeWithoutMatch();

    RCLCPP_DEBUG(
      logger,
      "[estimateTransforms] No overlap detected between %zu maps; "
      "all maps placed independently",
      images_.size());
    return true;
  }

  // // Experimental: should we keep only the best confidence match overall?
  // bool max_confidence_achieved_surpassed = false;
  // for (auto &match_info : pairwise_matches) {
  //   RCLCPP_INFO(logger, "[estimateTransforms] match info: %f", match_info.confidence);
  //   if (match_info.confidence > max_confidence_achieved_){
  //     max_confidence_achieved_surpassed = true;
  //     max_confidence_achieved_ = match_info.confidence;
  //   }
  // }
  // if (!max_confidence_achieved_surpassed) {
  //   RCLCPP_INFO(logger, "[estimateTransforms] Max confidence achieved not surpassed, not using matching");
  //   return false;
  // }
  // else
  //   RCLCPP_INFO(logger, "[estimateTransforms] Max confidence achieved surpassed, optimizing");


  /* estimate transform */
  RCLCPP_DEBUG(logger, "[estimateTransforms] calculating transforms in global reference frame");
  // note: currently used estimator never fails
  if (!(*estimator)(image_features, pairwise_matches, transforms)) {
    return false;
  }

  /* levmarq optimization */
  // openCV just accepts float transforms
  for (auto & transform : transforms) {
    transform.R.convertTo(transform.R, CV_32F);
  }
  RCLCPP_DEBUG(logger, "[estimateTransforms] optimizing global transforms");
  adjuster->setConfThresh(confidence);
  if (!(*adjuster)(image_features, pairwise_matches, transforms)) {
    RCLCPP_WARN(
      logger,
      "[estimateTransforms] Bundle adjusting failed. Could not estimate transforms.");
    return false;
  }

  transforms_.clear();
  transforms_.resize(images_.size());
  matched_.assign(images_.size(), false);

  size_t i = 0;
  for (auto & j : good_indices) {
    // we want to work with transforms as doubles
    transforms[i].R.convertTo(transforms_[static_cast<size_t>(j)], CV_64F);
    matched_[static_cast<size_t>(j)] = true;
    ++i;
  }

  // A correct alignment of grids with one resolution is rigid. Estimates
  // with a scale far from 1 are wrong matches; small scale errors are
  // removed so the grid drawn and the TF published (rotation only) agree.
  const double kMaxScaleError = 0.05;
  const double scale_error = makeTransformsRigid();
  if (scale_error > kMaxScaleError) {
    RCLCPP_INFO(
      logger,
      "[estimateTransforms] Rejected alignment with scale off by %.1f%% "
      "(inliers=%d, ratio=%.3f); keeping the last accepted alignment if there is one",
      100.0 * scale_error, best_inliers, best_ratio);
    placeWithoutMatch();
    return true;
  }

  // One wrong match must not move the merged map: an estimate that disagrees
  // with the alignment in use replaces it only after kConfirmEstimates
  // estimates in a row agree with it (the map really did need correcting).
  const double kMaxShiftM = 0.5;
  const double kMaxTurnRad = 8.0 * M_PI / 180.0;
  const int kConfirmEstimates = 3;
  // The first alignment has nothing to be checked against, so it needs two
  // estimates that agree with each other (weak passes in between do not
  // count against it).
  const int kConfirmFirst = 2;
  const std::vector<HeldAlignment> candidate = currentAlignment();
  const auto count_valid = [](const std::vector<HeldAlignment> & v) {
      return std::count_if(v.begin(), v.end(), [](const HeldAlignment & h) {return h.valid;});
    };
  if (count_valid(held_) < 2 && count_valid(candidate) >= 2) {
    bool same_as_pending = false;
    if (pending_.size() == candidate.size()) {
      const auto dp = alignmentDisagreement(pending_, candidate);
      same_as_pending = dp.first <= kMaxShiftM && dp.second <= kMaxTurnRad;
    }
    pending_count_ = same_as_pending ? pending_count_ + 1 : 1;
    pending_ = candidate;
    if (pending_count_ < kConfirmFirst) {
      RCLCPP_INFO(
        logger,
        "[estimateTransforms] First alignment found (inliers=%d, ratio=%.3f); "
        "waiting for one more agreeing estimate before using it",
        best_inliers, best_ratio);
      placeWithoutMatch();
      return true;
    }
  } else if (held_.size() == candidate.size()) {
    const auto d = alignmentDisagreement(held_, candidate);
    if (d.first > kMaxShiftM || d.second > kMaxTurnRad) {
      bool same_as_pending = false;
      if (pending_.size() == candidate.size()) {
        const auto dp = alignmentDisagreement(pending_, candidate);
        same_as_pending = dp.first <= kMaxShiftM && dp.second <= kMaxTurnRad;
      }
      pending_count_ = same_as_pending ? pending_count_ + 1 : 1;
      pending_ = candidate;
      if (pending_count_ < kConfirmEstimates) {
        RCLCPP_INFO(
          logger,
          "[estimateTransforms] New alignment differs from the one in use by "
          "%.2f m / %.1f deg (inliers=%d, ratio=%.3f); waiting for %d more "
          "agreeing estimate(s) before switching",
          d.first, d.second * 180.0 / M_PI, best_inliers, best_ratio,
          kConfirmEstimates - pending_count_);
        placeWithoutMatch();
        return true;
      }
      RCLCPP_WARN(
        logger,
        "[estimateTransforms] Switching to a new alignment %.2f m / %.1f deg "
        "from the previous one after %d agreeing estimates",
        d.first, d.second * 180.0 / M_PI, pending_count_);
    }
  }
  pending_.clear();
  pending_count_ = 0;

  rememberAlignment();

  // place any unmatched grids at non-overlapping fallback positions
  assignFallbackTransforms();

  size_t n_matched = matchedCount();
  if (n_matched < images_.size()) {
    RCLCPP_INFO(
      logger,
      "[estimateTransforms] %zu of %zu maps matched by features; "
      "%zu placed independently",
      n_matched, images_.size(), images_.size() - n_matched);
  }

  return true;
}

// checks whether given matrix is an identity, i.e. exactly appropriate Mat::eye
static inline bool isIdentity(const cv::Mat & matrix)
{
  if (matrix.empty()) {
    return false;
  }
  cv::MatExpr diff = matrix != cv::Mat::eye(matrix.size(), matrix.type());
  return cv::countNonZero(diff) == 0;
}

nav_msgs::msg::OccupancyGrid::SharedPtr MergingPipeline::composeGrids(rclcpp::Logger logger)
{
  // for checking states. Throws a rcpputils::IllegalStateException if the condition fails.
  rcpputils::check_true(images_.size() == transforms_.size());
  rcpputils::check_true(images_.size() == grids_.size());

  world_transforms_.clear();

  if (images_.empty()) {
    return nullptr;
  }

  RCLCPP_DEBUG(logger, "[composeGrids] warping grids");
  internal::GridWarper warper;
  std::vector<cv::Mat> imgs_warped;
  imgs_warped.reserve(images_.size());
  std::vector<cv::Rect> rois;
  rois.reserve(images_.size());

  for (size_t i = 0; i < images_.size(); ++i) {
    if (!transforms_[i].empty() && !images_[i].empty()) {
      imgs_warped.emplace_back();
      rois.emplace_back(
        warper.warp(images_[i], transforms_[i], imgs_warped.back()));
    }
  }

  if (imgs_warped.empty()) {
    return nullptr;
  }

  RCLCPP_DEBUG(logger, "[composeGrids] compositing result grid");
  nav_msgs::msg::OccupancyGrid::SharedPtr result;
  internal::GridCompositor compositor;
  result = compositor.compose(imgs_warped, rois);

  // set correct resolution to output grid. use resolution of identity (works
  // for estimated transforms), or any resolution (works for know_init_positions)
  // - in that case all resolutions should be the same.
  float any_resolution = 0.0;
  for (size_t i = 0; i < transforms_.size(); ++i) {
    // check if this transform is the reference frame
    if (isIdentity(transforms_[i])) {
      result->info.resolution = grids_[i]->info.resolution;
      break;
    }
    if (grids_[i]) {
      any_resolution = grids_[i]->info.resolution;
    }
  }
  if (result->info.resolution <= 0.f) {
    result->info.resolution = any_resolution;
  }

  // set grid origin to its centre
  result->info.origin.position.x =
    -(result->info.width / 2.0) * double(result->info.resolution);
  result->info.origin.position.y =
    -(result->info.height / 2.0) * double(result->info.resolution);
  result->info.origin.orientation.w = 1.0;

  // Record where each grid was actually drawn, as a metric transform from the
  // grid's own frame to the merged frame. GridWarper draws grid i with the
  // INVERSE of transforms_[i], and GridCompositor then shifts every grid by
  // the top-left corner of the union of the warped ROIs, so
  //   merged_px = inv(T_i) * grid_px - tl
  // Cell (c, r) is centred at origin + (c + 0.5, r + 0.5) * resolution in both
  // grids. All grids are assumed to share the merged resolution.
  cv::Point tl = rois.front().tl();
  for (const cv::Rect & roi : rois) {
    tl.x = std::min(tl.x, roi.x);
    tl.y = std::min(tl.y, roi.y);
  }
  const double res = static_cast<double>(result->info.resolution);
  geometry_msgs::msg::Transform unplaced;
  unplaced.rotation.w = 0.;  // zero quaternion marks "no placement"
  world_transforms_.assign(transforms_.size(), unplaced);
  for (size_t i = 0; i < transforms_.size(); ++i) {
    if (transforms_[i].empty() || images_[i].empty()) {
      continue;
    }
    cv::Mat H;
    cv::invertAffineTransform(transforms_[i].rowRange(0, 2), H);
    const double h00 = H.at<double>(0, 0), h01 = H.at<double>(0, 1);
    const double h10 = H.at<double>(1, 0), h11 = H.at<double>(1, 1);
    const double ox = grids_[i]->info.origin.position.x;
    const double oy = grids_[i]->info.origin.position.y;
    const double theta = std::atan2(h10, h00);

    geometry_msgs::msg::Transform & t = world_transforms_[i];
    t.translation.x = result->info.origin.position.x +
      (H.at<double>(0, 2) - tl.x + 0.5 - 0.5 * (h00 + h01)) * res -
      (h00 * ox + h01 * oy);
    t.translation.y = result->info.origin.position.y +
      (H.at<double>(1, 2) - tl.y + 0.5 - 0.5 * (h10 + h11)) * res -
      (h10 * ox + h11 * oy);
    t.translation.z = 0.;
    t.rotation.x = 0.;
    t.rotation.y = 0.;
    t.rotation.z = std::sin(theta * 0.5);
    t.rotation.w = std::cos(theta * 0.5);
  }

  return result;
}

size_t MergingPipeline::matchedCount() const
{
  size_t count = 0;
  for (bool m : matched_) {
    if (m) {++count;}
  }
  return count;
}

void MergingPipeline::rememberAlignment()
{
  if (matchedCount() < 2) {
    return;
  }
  held_ = currentAlignment();
}

std::vector<MergingPipeline::HeldAlignment> MergingPipeline::currentAlignment() const
{
  // transforms_ are in pixels of each grid and go stale as soon as a grid
  // grows (its origin moves), so keep the alignment in metres instead:
  //   P_common = R * P_grid + c,  with  [R | t] = inv(T_i),  c = t * res - R * O_i
  std::vector<HeldAlignment> out(transforms_.size());
  for (size_t i = 0; i < transforms_.size(); ++i) {
    if (i >= matched_.size() || !matched_[i] || transforms_[i].empty() || !grids_[i]) {
      continue;
    }
    cv::Mat H;
    cv::invertAffineTransform(transforms_[i].rowRange(0, 2), H);
    const double res = static_cast<double>(grids_[i]->info.resolution);
    const cv::Vec2d origin(
      grids_[i]->info.origin.position.x, grids_[i]->info.origin.position.y);
    HeldAlignment & held = out[i];
    held.R = cv::Matx22d(
      H.at<double>(0, 0), H.at<double>(0, 1),
      H.at<double>(1, 0), H.at<double>(1, 1));
    held.c = cv::Vec2d(H.at<double>(0, 2), H.at<double>(1, 2)) * res - held.R * origin;
    held.valid = true;
  }
  return out;
}

std::pair<double, double> MergingPipeline::alignmentDisagreement(
  const std::vector<HeldAlignment> & a,
  const std::vector<HeldAlignment> & b) const
{
  double worst_m = 0.0;
  double worst_rad = 0.0;
  const size_t n = std::min({a.size(), b.size(), grids_.size()});
  for (size_t i = 0; i < n; ++i) {
    if (!a[i].valid || !b[i].valid) {
      continue;
    }
    for (size_t j = 0; j < n; ++j) {
      if (j == i || !a[j].valid || !b[j].valid || !grids_[j]) {
        continue;
      }
      // Where the centre of grid j lands in grid i's frame, under each.
      const auto & info = grids_[j]->info;
      const cv::Vec2d centre(
        info.origin.position.x + 0.5 * info.width * info.resolution,
        info.origin.position.y + 0.5 * info.height * info.resolution);
      const cv::Vec2d pa = a[i].R.inv() * (a[j].R * centre + a[j].c - a[i].c);
      const cv::Vec2d pb = b[i].R.inv() * (b[j].R * centre + b[j].c - b[i].c);
      worst_m = std::max(worst_m, cv::norm(pa - pb));
      const double ta = std::atan2(a[j].R(1, 0), a[j].R(0, 0)) -
        std::atan2(a[i].R(1, 0), a[i].R(0, 0));
      const double tb = std::atan2(b[j].R(1, 0), b[j].R(0, 0)) -
        std::atan2(b[i].R(1, 0), b[i].R(0, 0));
      worst_rad = std::max(
        worst_rad, std::abs(std::atan2(std::sin(ta - tb), std::cos(ta - tb))));
    }
  }
  return {worst_m, worst_rad};
}

double MergingPipeline::makeTransformsRigid()
{
  double worst = 0.0;
  for (size_t i = 0; i < transforms_.size(); ++i) {
    if (i >= matched_.size() || !matched_[i] || transforms_[i].empty() ||
      images_[i].empty())
    {
      continue;
    }
    // H: grid pixels -> merged pixels (what the warper draws).
    cv::Mat H;
    cv::invertAffineTransform(transforms_[i].rowRange(0, 2), H);
    const double a = H.at<double>(0, 0), b = H.at<double>(0, 1);
    const double c = H.at<double>(1, 0), d = H.at<double>(1, 1);
    const double scale = std::sqrt(std::abs(a * d - b * c));
    if (!(scale > 1e-6)) {
      worst = std::max(worst, 1.0);
      continue;
    }
    worst = std::max(worst, std::abs(scale - 1.0));
    const double theta = std::atan2(c - b, a + d);
    const double cs = std::cos(theta), sn = std::sin(theta);
    // Keep the grid centre where it was.
    const double px = 0.5 * images_[i].cols, py = 0.5 * images_[i].rows;
    const double qx = a * px + b * py + H.at<double>(0, 2);
    const double qy = c * px + d * py + H.at<double>(1, 2);
    cv::Mat R = (cv::Mat_<double>(2, 3) <<
      cs, -sn, qx - (cs * px - sn * py),
      sn, cs, qy - (sn * px + cs * py));
    cv::Mat forward = transforms_[i].rowRange(0, 2);
    cv::invertAffineTransform(R, forward);
  }
  return worst;
}

void MergingPipeline::placeWithoutMatch()
{
  transforms_.clear();
  transforms_.resize(images_.size());
  matched_.assign(images_.size(), false);

  // Reuse the last accepted alignment for the grids that had one, rebuilt for
  // the grids as they are now. One weak estimate must not throw away a good
  // alignment: the grid and TF would jump apart and back on every such pass.
  size_t reusable = 0;
  if (held_.size() == images_.size()) {
    for (size_t i = 0; i < images_.size(); ++i) {
      if (held_[i].valid && !images_[i].empty()) {
        ++reusable;
      }
    }
  }
  if (reusable >= 2) {
    for (size_t i = 0; i < images_.size(); ++i) {
      if (!held_[i].valid || images_[i].empty()) {
        continue;
      }
      const double res = static_cast<double>(grids_[i]->info.resolution);
      const cv::Vec2d origin(
        grids_[i]->info.origin.position.x, grids_[i]->info.origin.position.y);
      const cv::Vec2d t = (held_[i].c + held_[i].R * origin) * (1.0 / res);
      cv::Mat H = (cv::Mat_<double>(2, 3) <<
        held_[i].R(0, 0), held_[i].R(0, 1), t[0],
        held_[i].R(1, 0), held_[i].R(1, 1), t[1]);
      transforms_[i] = cv::Mat::eye(3, 3, CV_64F);
      cv::Mat forward = transforms_[i].rowRange(0, 2);
      cv::invertAffineTransform(H, forward);
      matched_[i] = true;
    }
  } else {
    // nothing to reuse: identity for the first non-empty grid (reference frame)
    for (size_t i = 0; i < images_.size(); ++i) {
      if (!images_[i].empty()) {
        transforms_[i] = cv::Mat::eye(3, 3, CV_64F);
        break;
      }
    }
  }

  // place remaining grids at non-overlapping positions
  assignFallbackTransforms();
}

void MergingPipeline::assignFallbackTransforms()
{
  // Gap between independent maps in pixels.  At 0.05 m/pixel this is 5 m,
  // large enough to keep maps visually separate in the merged grid.
  constexpr int GAP_PIXELS = 100;

  // find the rightmost edge of any grid that already has a transform
  double max_right = 0.0;
  for (size_t i = 0; i < transforms_.size(); ++i) {
    if (!transforms_[i].empty() && !images_[i].empty()) {
      double tx = transforms_[i].at<double>(0, 2);
      double right = tx + static_cast<double>(images_[i].cols);
      if (right > max_right) {
        max_right = right;
      }
    }
  }

  // place every grid that still has an empty transform to the right
  double offset_x = max_right + GAP_PIXELS;
  for (size_t i = 0; i < transforms_.size(); ++i) {
    if (transforms_[i].empty() && !images_[i].empty()) {
      transforms_[i] = cv::Mat::eye(3, 3, CV_64F);
      transforms_[i].at<double>(0, 2) = offset_x;
      offset_x += static_cast<double>(images_[i].cols) + GAP_PIXELS;
    }
  }
}

std::vector<geometry_msgs::msg::Transform> MergingPipeline::getTransforms() const
{
  std::vector<geometry_msgs::msg::Transform> result;
  result.reserve(transforms_.size());

  for (auto & transform : transforms_) {
    if (transform.empty()) {
      result.emplace_back();
      continue;
    }

    rcpputils::require_true(transform.type() == CV_64F);
    geometry_msgs::msg::Transform ros_transform;
    ros_transform.translation.x = transform.at<double>(0, 2);
    ros_transform.translation.y = transform.at<double>(1, 2);
    ros_transform.translation.z = 0.;

    // our rotation is in fact only 2D, thus quaternion can be simplified
    double a = transform.at<double>(0, 0);
    double b = transform.at<double>(1, 0);
    ros_transform.rotation.w = std::sqrt(2. + 2. * a) * 0.5;
    ros_transform.rotation.x = 0.;
    ros_transform.rotation.y = 0.;
    ros_transform.rotation.z = std::copysign(std::sqrt(2. - 2. * a) * 0.5, b);

    result.push_back(ros_transform);
  }

  return result;
}

}  // namespace combine_grids
