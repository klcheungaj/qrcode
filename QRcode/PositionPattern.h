#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

using namespace std;
using namespace cv;

struct lineInfo
{
	Vec4f line;
	int ends[2];
};

class PositionPattern
{
public:
	PositionPattern(vector<Point> &contour, int width);
	Vec4i detectTimePattern(PositionPattern &rect2, const Mat &binaryImage, Mat& drawingLine, float thresh = 5);
	bool getBoundingLine(Vec4f &bounding);
	Point2f getBoundingPoint();
	bool findTimePatternPairs(const PositionPattern &rect2, vector<vector<Point2f>> &minDistanceCorners);
	Point2f operator[] (int x) const;
	vector<int>& getTimePatternSidePoint();
	int getTimePatternPoint();
	void setTimePatternPoint(int index);
	bool isEmpty() const;
	int getSize() const;
	int getWidth() const;
	int getNumOfTimePatternPoint() const;
	void addNumOfTimePatternPoint();
	~PositionPattern();

private:
	vector<Point2f> corners;
	vector<lineInfo> lines;
    vector<int> timePatternSidePoint;
	int timePatternPoint;
	int numOfTimePatternPoint;
	int width;
};

Vec2f twoPt2vec2f(cv::Point pt1, Point pt2);

vector<Point2f> lineToPointPair(Vec2f line);

Point2f computeIntersect(const Vec2f line1, const Vec2f line2);

Point2f computeIntersect(const Vec4f &line1, const Vec4f &line2);

inline float computeDistance(const Point &i, const Point &j);

inline float computeDistance(const Point2f &i, const Point2f &j);

bool DescendingSize(vector<Point> &i, vector<Point> &j);

bool DescendingSlope(Vec4f &i, Vec4f &j);

/* don't check empty */
/* assume same index */
bool DescendingFistVal(vector<int> &i, vector<int> &j);


float computeMean(const std::vector<int> &numbers);

float computeVariance(const std::vector<int> &numbers);

bool findTimePatternPairs(const vector<Point2f> &rect1, const vector<Point2f> &rect2, vector<vector<Point2f>> &minDistanceCorners);

bool detectTimePattern(const Mat &binaryImage, Point2f startPt, Point2f endPt, float thresh = 10);