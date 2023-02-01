#pragma once

#include <cmath>
#include <vector>
//#include <boost/array.hpp>
#include <Eigen/Dense>
#include "line_finder/utilities.h"
#include "line_finder/line.h"

namespace line_finder
{

class LineExtraction
{

public:
  // Constructor / destructor
  LineExtraction();
  ~LineExtraction();
  // Run
  void extractLines(std::vector<Line>&, bool debug);
  // Data setting
  void setCachedData(const std::vector<double>&, const std::vector<double>&,
                     const std::vector<double>&, const std::vector<unsigned int>&);
  void setRangeData(const std::vector<double>&, bool debug);
  // Parameter setting
  void setBearingVariance(double);
  void setRangeVariance(double);
  void setLeastSqAngleThresh(double);
  void setLeastSqRadiusThresh(double);
  void setMaxLineGap(double);
  void setMinLineLength(double);
  void setMinLinePoints(unsigned int);
  void setMinRange(double);
  void setMaxRange(double);
  void setMinSplitDist(double);
  void setOutlierDist(double);

private:
  // Data structures
  CachedData c_data_;
  RangeData r_data_;
  Params params_;
  // Indices after filtering
  std::vector<unsigned int> filtered_indices_;
  // Line data
  std::vector<Line> lines_;
  // Methods
  double chiSquared(const Eigen::Vector2d&, const Eigen::Matrix2d&,
                    const Eigen::Matrix2d&);
  double distBetweenPoints(unsigned int index_1, unsigned int index_2);
  void   filterCloseAndFarPoints(bool debug);
  void   filterOutlierPoints(bool debug);
  void   filterLines();
  void   mergeLines();
  void   split(const std::vector<unsigned int>&);
};

} // namespace line_finder

