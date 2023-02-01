#include "line_finder/line_extraction.h"

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>

namespace line_finder {

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtraction::LineExtraction() {}

LineExtraction::~LineExtraction() {}

///////////////////////////////////////////////////////////////////////////////
// Main run function
///////////////////////////////////////////////////////////////////////////////
void LineExtraction::extractLines(std::vector<Line>& lines, bool debug) {
  // Resets
  filtered_indices_ = c_data_.indices;
  lines_.clear();

  // Filter indices
  filterCloseAndFarPoints(debug);
  filterOutlierPoints(debug);

  // Return no lines if not enough points left
  if (filtered_indices_.size() <=
      std::max(params_.min_line_points, static_cast<unsigned int>(3))) {
    return;
  }

  // Split indices into lines and filter out short and sparse lines
  split(filtered_indices_);
  filterLines();

  // Fit each line using least squares and merge colinear lines
  for (std::vector<Line>::iterator it = lines_.begin(); it != lines_.end();
       ++it) {
    it->leastSqFit();
  }

  // If there is more than one line, check if lines should be merged based on
  // the merging criteria
  if (lines_.size() > 1) {
    mergeLines();
  }

  lines = lines_;
}

///////////////////////////////////////////////////////////////////////////////
// Data setting
///////////////////////////////////////////////////////////////////////////////
void LineExtraction::setCachedData(const std::vector<double>& bearings,
                                   const std::vector<double>& cos_bearings,
                                   const std::vector<double>& sin_bearings,
                                   const std::vector<unsigned int>& indices) {
  c_data_.bearings = bearings;
  c_data_.cos_bearings = cos_bearings;
  c_data_.sin_bearings = sin_bearings;
  c_data_.indices = indices;
}

void LineExtraction::setRangeData(const std::vector<double>& ranges,
                                  bool debug) {
  std::stringstream explanation;
  r_data_.ranges = ranges;
  r_data_.xs.clear();
  r_data_.ys.clear();
  if (debug) {
    explanation << "LineExtraction::setRangeData" << std::endl;
  }

  size_t i = 0;
  for (std::vector<unsigned int>::const_iterator cit = c_data_.indices.begin();
       cit != c_data_.indices.end(); ++cit) {
    double x = c_data_.cos_bearings[*cit] * ranges[*cit];
    double y = c_data_.sin_bearings[*cit] * ranges[*cit];
    r_data_.xs.push_back(x);
    r_data_.ys.push_back(y);
    if (debug) {
      explanation << "[" << i++ << "] xs: " << x << ", ys: " << y << std::endl;
    }
  }

  if (debug) {
    std::cout << explanation.str();
  }
}

///////////////////////////////////////////////////////////////////////////////
// Parameter setting
///////////////////////////////////////////////////////////////////////////////
void LineExtraction::setBearingVariance(double value) {
  params_.bearing_var = value;
}

void LineExtraction::setRangeVariance(double value) {
  params_.range_var = value;
}

void LineExtraction::setLeastSqAngleThresh(double value) {
  params_.least_sq_angle_thresh = value;
}

void LineExtraction::setLeastSqRadiusThresh(double value) {
  params_.least_sq_radius_thresh = value;
}

void LineExtraction::setMaxLineGap(double value) {
  params_.max_line_gap = value;
}

void LineExtraction::setMinLineLength(double value) {
  params_.min_line_length = value;
}

void LineExtraction::setMinLinePoints(unsigned int value) {
  params_.min_line_points = value;
}

void LineExtraction::setMinRange(double value) { params_.min_range = value; }

void LineExtraction::setMaxRange(double value) { params_.max_range = value; }

void LineExtraction::setMinSplitDist(double value) {
  params_.min_split_dist = value;
}

void LineExtraction::setOutlierDist(double value) {
  params_.outlier_dist = value;
}

///////////////////////////////////////////////////////////////////////////////
// Utility methods
///////////////////////////////////////////////////////////////////////////////
double LineExtraction::chiSquared(const Eigen::Vector2d& dL,
                                  const Eigen::Matrix2d& P_1,
                                  const Eigen::Matrix2d& P_2) {
  return dL.transpose() * (P_1 + P_2).inverse() * dL;
}

double LineExtraction::distBetweenPoints(unsigned int index_1,
                                         unsigned int index_2) {
  return sqrt(pow(r_data_.xs[index_1] - r_data_.xs[index_2], 2) +
              pow(r_data_.ys[index_1] - r_data_.ys[index_2], 2));
}

///////////////////////////////////////////////////////////////////////////////
// Filtering points
///////////////////////////////////////////////////////////////////////////////
void LineExtraction::filterCloseAndFarPoints(bool debug) {
  std::vector<unsigned int> output;
  size_t i = 0;
  for (std::vector<unsigned int>::const_iterator cit =
           filtered_indices_.begin();
       cit != filtered_indices_.end(); ++cit) {
    const double& range = r_data_.ranges[*cit];
    if (range >= params_.min_range && range <= params_.max_range) {
      output.push_back(*cit);
    } else if (debug) {
      std::cout << " LineExtraction::filterCloseAndFarPoints [" << i
                << "] range: " << range << " not in range of: ["
                << params_.min_range << " .. " << params_.max_range << "]"
                << std::endl;
    }
    i++;
  }
  filtered_indices_ = output;
}

void LineExtraction::filterOutlierPoints(bool debug) {
  if (filtered_indices_.size() < 3) {
    if (debug) {
      std::cout << "[LineExtraction::filterOutlierPoints] size: "
                << filtered_indices_.size() << " is < 3, ignoring" << std::endl;
    }
    return;
  }

  std::vector<unsigned int> output;
  unsigned int p_i, p_j, p_k;
  for (std::size_t i = 0; i < filtered_indices_.size(); ++i) {
    // Get two closest neighbours

    p_i = filtered_indices_[i];
    if (i == 0) {
      // first point
      p_j = filtered_indices_[i + 1];
      p_k = filtered_indices_[i + 2];
    } else if (i == filtered_indices_.size() - 1) {
      // last point
      p_j = filtered_indices_[i - 1];
      p_k = filtered_indices_[i - 2];
    } else {
      // middle points
      p_j = filtered_indices_[i - 1];
      p_k = filtered_indices_[i + 1];
    }

    // Check if point is an outlier
    if (fabs(r_data_.ranges[p_i] - r_data_.ranges[p_j]) >
            params_.outlier_dist &&
        fabs(r_data_.ranges[p_i] - r_data_.ranges[p_k]) >
            params_.outlier_dist) {
      // Check if it is close to line connecting its neighbours
      std::vector<unsigned int> line_indices;
      line_indices.push_back(p_j);
      line_indices.push_back(p_k);
      Line line(c_data_, r_data_, params_, line_indices);
      line.endpointFit();
      if (line.distToPoint(p_i) > params_.min_split_dist) {
        if (debug) {
          std::cout << "[LineExtraction::filterOutlierPoints] [" << i
                    << "] point is an outlier"
                    << ", p_i: " << p_i
                    << ", p_i range: " << r_data_.ranges[p_i]
                    << ", p_j: " << p_j
                    << ", p_j range: " << r_data_.ranges[p_j]
                    << ", p_k: " << p_k
                    << ", p_k range: " << r_data_.ranges[p_k]
                    << ", outlier_dist: " << params_.outlier_dist
                    << ", j-k line distToPoint i: " << line.distToPoint(p_i)
                    << " i > min_split_dist: params_.min_split_dist"
                    << std::endl;
        }
        continue;  // point is an outlier
      }
    }

    output.push_back(p_i);
  }

  filtered_indices_ = output;
}

///////////////////////////////////////////////////////////////////////////////
// Filtering and merging lines
///////////////////////////////////////////////////////////////////////////////
void LineExtraction::filterLines() {  // middle points
  std::vector<Line> output;
  for (std::vector<Line>::const_iterator cit = lines_.begin();
       cit != lines_.end(); ++cit) {
    if (cit->length() >= params_.min_line_length &&
        cit->numPoints() >= params_.min_line_points) {
      output.push_back(*cit);
    }
  }
  lines_ = output;
}

void LineExtraction::mergeLines() {
  std::vector<Line> merged_lines;

  for (std::size_t i = 1; i < lines_.size(); ++i) {
    // Get L, P_1, P_2 of consecutive lines
    Eigen::Vector2d L_1(lines_[i - 1].getRadius(), lines_[i - 1].getAngle());
    Eigen::Vector2d L_2(lines_[i].getRadius(), lines_[i].getAngle());
    Eigen::Matrix2d P_1;
    P_1 << lines_[i - 1].getCovariance()[0], lines_[i - 1].getCovariance()[1],
        lines_[i - 1].getCovariance()[2], lines_[i - 1].getCovariance()[3];
    Eigen::Matrix2d P_2;
    P_2 << lines_[i].getCovariance()[0], lines_[i].getCovariance()[1],
        lines_[i].getCovariance()[2], lines_[i].getCovariance()[3];

    // Merge lines if chi-squared distance is less than 3
    if (chiSquared(L_1 - L_2, P_1, P_2) < 3) {
      // Get merged angle, radius, and covariance
      Eigen::Matrix2d P_m = (P_1.inverse() + P_2.inverse()).inverse();
      Eigen::Vector2d L_m = P_m * (P_1.inverse() * L_1 + P_2.inverse() * L_2);
      // Populate new line with these merged parameters
      boost::array<double, 4> cov;
      cov[0] = P_m(0, 0);
      cov[1] = P_m(0, 1);
      cov[2] = P_m(1, 0);
      cov[3] = P_m(1, 1);
      std::vector<unsigned int> indices;
      const std::vector<unsigned int>& ind_1 = lines_[i - 1].getIndices();
      const std::vector<unsigned int>& ind_2 = lines_[i].getIndices();
      indices.resize(ind_1.size() + ind_2.size());
      indices.insert(indices.end(), ind_1.begin(), ind_1.end());
      indices.insert(indices.end(), ind_2.begin(), ind_2.end());
      Line merged_line(L_m[1], L_m[0], cov, lines_[i - 1].getStart(),
                       lines_[i].getEnd(), indices);
      // Project the new endpoints
      merged_line.projectEndpoints();
      lines_[i] = merged_line;
    } else {
      merged_lines.push_back(lines_[i - 1]);
    }

    if (i == lines_.size() - 1) {
      merged_lines.push_back(lines_[i]);
    }
  }
  lines_ = merged_lines;
}

///////////////////////////////////////////////////////////////////////////////
// Splitting points into lines
///////////////////////////////////////////////////////////////////////////////
void LineExtraction::split(const std::vector<unsigned int>& indices) {
  // Don't split if only a single point (only occurs when orphaned by gap)
  if (indices.size() <= 1) {
    return;
  }

  Line line(c_data_, r_data_, params_, indices);
  line.endpointFit();
  double dist_max = 0;
  double gap_max = 0;
  double dist, gap;
  int i_max, i_gap;

  // Find the farthest point and largest gap
  for (std::size_t i = 1; i < indices.size() - 1; ++i) {
    dist = line.distToPoint(indices[i]);
    if (dist > dist_max) {
      dist_max = dist;
      i_max = i;
    }
    gap = distBetweenPoints(indices[i], indices[i + 1]);
    if (gap > gap_max) {
      gap_max = gap;
      i_gap = i;
    }
  }

  // Check for gaps at endpoints
  double gap_start = distBetweenPoints(indices[0], indices[1]);
  if (gap_start > gap_max) {
    gap_max = gap_start;
    i_gap = 1;
  }
  double gap_end = distBetweenPoints(indices.rbegin()[1], indices.rbegin()[0]);
  if (gap_end > gap_max) {
    gap_max = gap_end;
    i_gap = indices.size() - 1;
  }

  // Check if line meets requirements or should be split
  if (dist_max < params_.min_split_dist && gap_max < params_.max_line_gap) {
    lines_.push_back(line);
  } else {
    int i_split = dist_max >= params_.min_split_dist ? i_max : i_gap;
    std::vector<unsigned int> first_split(&indices[0], &indices[i_split - 1]);
    std::vector<unsigned int> second_split(&indices[i_split], &indices.back());
    split(first_split);
    split(second_split);
  }
}

}  // namespace line_finder
