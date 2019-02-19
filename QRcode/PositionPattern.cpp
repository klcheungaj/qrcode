#include "PositionPattern.h"
#include <iostream>
#include <math.h>
#include <algorithm>

PositionPattern::PositionPattern(vector<Point> &contour, int width) :
	numOfTimePatternPoint(0),
	width(width)
{
	if (!contour.empty())
	{
		vector<Point> polyContour;
		approxPolyDP(contour, polyContour, 3, true);
		vector<Point> hull;
		convexHull(Mat(polyContour), hull, true);
		vector<vector<Point>> lineSegments(hull.size());

		if (hull.size() <= 6)
		{
			for (int i = 0; i < hull.size(); i++)
			{
				vector<Point>::iterator start = find(contour.begin(), contour.end(), hull[i]);
				vector<Point>::iterator end;
				if (i == hull.size() - 1)
					end = find(contour.begin(), contour.end(), hull[0]);
				else
					end = find(contour.begin(), contour.end(), hull[i + 1]);

				if (start != contour.end() && end != contour.end())
				{
					int numOfPoint = abs(end - start) + 1;
					int numOfPoint_inv = contour.size() - numOfPoint + 2;
					if (numOfPoint_inv > numOfPoint)
					{
						lineSegments[i].resize(numOfPoint);

						if (start > end)
							swap(start, end);
						copy(start, end + 1, lineSegments[i].begin());
					}
					else
					{
						lineSegments[i].resize(numOfPoint_inv);
						if (start < end)
							swap(start, end);
						copy(start, contour.end(), lineSegments[i].begin());
						copy(contour.begin(), end + 1, lineSegments[i].begin() + (contour.end() - start));
					}

				}
			}

			/* show segment		*/
			//for (int j = 0; j < lineSegments.size(); j++)
			//{
			//	Mat drawing = Mat::zeros(size, CV_8UC1);
			//	for (Point pt : lineSegments[j])
			//		circle(drawing, pt, 5, (255, 255, 255), 2);
			//	imshow(to_string(contour.size())+"<<<<<<<<<<" + to_string(j) + " line segment: ", drawing);
			//}

			if (lineSegments.size() >= 4)
			{
				if (lineSegments.size() > 4)
				{
					sort(lineSegments.begin(), lineSegments.end(), DescendingSize);
					lineSegments.erase(lineSegments.begin() + 4, lineSegments.end());
				}

				/* fitting line */
				vector<Vec4f> boundingLine(4);
				for (int k = 0; k < lineSegments.size(); k++)
				{
					fitLine(lineSegments[k], boundingLine[k], cv::DIST_L1, 0, 0.01, 0.01);
				}

				/* drawing */
				//Mat drawing = Mat::zeros(size, CV_8UC1);
				//for (Vec4f line : boundingLine)
				//{
				//	cv::Point point0;
				//	point0.x = line[2];
				//	point0.y = line[3];
				//	double k = line[1] / line[0];
				//	cv::Point point1, point2;
				//	point1.x = 0;
				//	point1.y = k * (0 - point0.x) + point0.y;
				//	point2.x = 640;
				//	point2.y = k * (640 - point0.x) + point0.y;
				//	cv::line(drawing, point1, point2, cv::Scalar(255, 255, 255), 2, 8, 0);
				//}
				//imshow(to_string(contour.size()) + " line", drawing);

				sort(boundingLine.begin(), boundingLine.end(), DescendingSlope);
				this->corners.resize(4);
				for (int i = 0; i < 2; i++)
				{
					corners[0 + 2 * i] = computeIntersect(boundingLine[i], boundingLine[2]);
					//if corners.
					corners[1 + 2 * i] = computeIntersect(boundingLine[i], boundingLine[3]);
				}



				lines.resize(4);
				lines[0].line = boundingLine[0];
				lines[0].ends[0] = 0;
				lines[0].ends[1] = 1;
				lines[1].line = boundingLine[1];
				lines[1].ends[0] = 2;
				lines[1].ends[1] = 3;
				lines[2].line = boundingLine[2];
				lines[2].ends[0] = 0;
				lines[2].ends[1] = 2;
				lines[3].line = boundingLine[3];
				lines[3].ends[0] = 1;
				lines[3].ends[1] = 3;
				/* draw corners */
			/*	for (Point pt : corners)
					circle(drawContour, pt, 5, (255, 255, 255), 2);
				imshow("corners", drawContour);*/
			}
		}
	}
}


PositionPattern::~PositionPattern()
{

}

/* for minDistanceCorners[a][b] 
   a = 0 and a = 1 are 2 diferent pairs
   b = 0 comes from rect1(this)
   b = 1 comes from rect2.
*/
bool PositionPattern::findTimePatternPairs(const PositionPattern &rect2, vector<vector<Point2f>> &minDistanceCorners)
{
	if (!(this->isEmpty()) && !(rect2.isEmpty()))
	{
		if (this->getSize() > 2  && rect2.getSize() > 2)
		{
			minDistanceCorners.resize(2);
			minDistanceCorners[0].resize(2);
			minDistanceCorners[1].resize(2);
			float temp = 1E6;
			float minDistance = 1E6;
			int rect1_index, rect2_index;
			Point2f pt1;
			Point2f pt2;
			for (int i = 0; i < lines.size(); i++)
			{
				for (int j = 0; j < rect2.lines.size(); j++)
				{
					Point2f pt1((corners[lines[i].ends[0]].x + corners[lines[i].ends[1]].x) / 2, (corners[lines[i].ends[0]].y + corners[lines[i].ends[1]].y) / 2);
					Point2f pt2((rect2[rect2.lines[j].ends[0]].x + rect2[rect2.lines[j].ends[1]].x) / 2, (rect2[rect2.lines[j].ends[0]].y + rect2[rect2.lines[j].ends[1]].y) / 2);
					temp = computeDistance(pt1, pt2);
					if (temp < minDistance)
					{
						minDistance = temp;
						rect1_index = i;
						rect2_index = j;
					}
				}
			}

			float closePtDist = 1E6;
			for (int i = 0; i < 2; i++)
			{
				for (int j = 0; j < 2; j++)
				{
					float temp = computeDistance(corners[lines[rect1_index].ends[i]], rect2[rect2.lines[rect2_index].ends[j]]);
					if (temp < closePtDist)
					{
						closePtDist = temp;
						minDistanceCorners[1][0] = corners[lines[rect1_index].ends[i]];
						minDistanceCorners[1][1] = rect2[rect2.lines[rect2_index].ends[j]];
						minDistanceCorners[0][0] = corners[lines[rect1_index].ends[1-i]];
						minDistanceCorners[0][1] = rect2[rect2.lines[rect2_index].ends[1-j]];
					}
				}
			}
			return true;
		}
	}
	return false;
}
 

Vec4i PositionPattern::detectTimePattern(PositionPattern &rect2,  const Mat &binaryImage, Mat& drawingLine, float thresh)
{

	vector<vector<Point2f>> minDistanceCorners;
	if (findTimePatternPairs(rect2, minDistanceCorners))
	{
		for(int twoPair = 0; twoPair<2 ; twoPair++)
		{
			Point2f rect1_side[2], rect2_side[2];
			if (!twoPair)
			{
				rect1_side[0] = minDistanceCorners[0][0];
				rect1_side[1] = minDistanceCorners[1][0];
				rect2_side[0] = minDistanceCorners[0][1];
				rect2_side[1] = minDistanceCorners[1][1];
			}
			else
			{
				rect1_side[1] = minDistanceCorners[0][0];
				rect1_side[0] = minDistanceCorners[1][0];
				rect2_side[0] = minDistanceCorners[0][1];
				rect2_side[1] = minDistanceCorners[1][1];
			}
			for (int a = 0; a < 2; a++)
			{
				Point2f timePatternA(rect1_side[a].x + (rect1_side[1 - a].x - rect1_side[a].x) / 14, rect1_side[a].y + (rect1_side[1 - a].y - rect1_side[a].y) / 14);
				Point2f timePatternB(rect2_side[a].x + (rect2_side[1 - a].x - rect2_side[a].x) / 14, rect2_side[a].y + (rect2_side[1 - a].y - rect2_side[a].y) / 14);

				/* 50 is to length filter*/
				if (timePatternA.x > 0 && timePatternA.y > 0 && timePatternB.x > 0 && timePatternB.y > 0 && computeDistance(timePatternA, timePatternB) > 50)
				{
					/* get vale along line except start & end */
					cv::LineIterator lit(binaryImage, timePatternA, timePatternB, 8);
					vector<Point> linePts;
					linePts.reserve(lit.count);
					for (int i = 0; i < lit.count; i++, lit++)
					{
						linePts.push_back(lit.pos());
						//cout << static_cast<int>(adapThresh.at<uchar>(pts[i])) <<",";
					}


					if (!linePts.empty())
					{
						unsigned char contColor = binaryImage.at<unsigned char>(*linePts.begin());
						int count = 0;
						vector<int> pattern;
						pattern.reserve(20);
						for (Point pt : linePts)
						{
							if (pt.x > 0 && pt.y > 0)
							{
								if ((binaryImage.at<unsigned char>(pt)) == contColor)
								{
									++count;
								}
								else
								{
									pattern.push_back(count);
									count = 1;
									contColor = binaryImage.at<unsigned char>(pt);
								}
							}
						}
						if (count > 1)
							pattern.push_back(count);

						if (!pattern.empty())
							pattern.pop_back();
						if (!pattern.empty())
							pattern.erase(pattern.begin());

						line(drawingLine, timePatternA, timePatternB, cv::Scalar(128, 128, 128), 1, 8, 0);

						float var = computeVariance(pattern);
						cout << var << endl;
						if (var < thresh)
						{
							/* search and save corners index in vector */
							for (int j = 0; j < corners.size(); j++)
							{
								if ((static_cast<int>(rect1_side[a].x) == static_cast<int>(corners[j].x)) && ((static_cast<int>(rect1_side[a].y) == static_cast<int>(corners[j].y))))
								{
									++numOfTimePatternPoint;
									timePatternPoint = j;
									if (find(timePatternSidePoint.begin(), timePatternSidePoint.end(), j) == timePatternSidePoint.end())
										timePatternSidePoint.push_back(j);
								}
								if ((static_cast<int>(rect1_side[1 - a].x) == static_cast<int>(corners[j].x)) && ((static_cast<int>(rect1_side[1 - a].y) == static_cast<int>(corners[j].y))))
								{
									if (find(timePatternSidePoint.begin(), timePatternSidePoint.end(), j) == timePatternSidePoint.end())
										timePatternSidePoint.push_back(j);
								}
							}


							for (int j = 0; j < rect2.getSize(); j++)
							{
								if ((static_cast<int>(rect2_side[a].x) == static_cast<int>(rect2[j].x)) && ((static_cast<int>(rect2_side[a].y) == static_cast<int>(rect2[j].y))))
								{
									rect2.addNumOfTimePatternPoint();
									rect2.setTimePatternPoint(j);
									if (find(rect2.getTimePatternSidePoint().begin(), rect2.getTimePatternSidePoint().end(), j) == rect2.getTimePatternSidePoint().end())
										rect2.getTimePatternSidePoint().push_back(j);
								}
								if ((static_cast<int>(rect2_side[1 - a].x) == static_cast<int>(rect2[j].x)) && ((static_cast<int>(rect2_side[1 - a].y) == static_cast<int>(rect2[j].y))))
								{
									if (find(rect2.getTimePatternSidePoint().begin(), rect2.getTimePatternSidePoint().end(), j) == rect2.getTimePatternSidePoint().end())
										rect2.getTimePatternSidePoint().push_back(j);
								}
							}


							Vec4i result(rect1_side[a].x, rect1_side[a].y, rect2_side[a].x, rect2_side[a].y);
							return result;
						}
					}

				}
			}
		}
	}

	Vec4i result(0, 0, 0, 0);
	return result;
}

bool PositionPattern::getBoundingLine(Vec4f &bounding)
{
	if (numOfTimePatternPoint == 1)
	{
		for (lineInfo line : lines)
		{
			if (timePatternSidePoint.size() == 2)
			{
				if ((line.ends[0] != timePatternSidePoint[0]) && (line.ends[0] != timePatternSidePoint[1]) && (line.ends[1] != timePatternSidePoint[0]) && (line.ends[1] != timePatternSidePoint[1]))
				{
					bounding = line.line;
					return true;
				}
			}
		}
	}
	else
	{
		return false;
	}
}

void PositionPattern::setTimePatternPoint(int index)
{
	timePatternPoint = index;
}

int PositionPattern::getWidth() const
{
	return width;
}

Point2f PositionPattern::getBoundingPoint()
{
	if (numOfTimePatternPoint == 1)
	{
		vector<int> index;
		for (int i = 0; i < lines.size(); i++)
		{
			if ((lines[i].ends[0] != timePatternPoint) && (lines[i].ends[1] != timePatternPoint))
			{
				if (index.empty())
				{
					index.push_back(lines[i].ends[0]);
					index.push_back(lines[i].ends[1]);
				}
				else
				{
					if (index[0] == lines[i].ends[0])
						return corners[index[0]];
					if (index[0] == lines[i].ends[1])
						return corners[index[0]];
					if (index[1] == lines[i].ends[0])
						return corners[index[1]];
					if (index[1] == lines[i].ends[1])
						return corners[index[1]];
				}
			}
		}
	}
	else if (numOfTimePatternPoint == 2)
	{
		for (int i = 0; i < corners.size(); i++)
		{
			bool findCorners = false;
			for (int j = 0; j < timePatternSidePoint.size(); j++)
			{
				if (timePatternSidePoint[j] == i)
					findCorners = true;
			}

			if (!findCorners)
				return corners[i];
		}
	}
	else
	{
		return Point2f(0, 0);
	}
}

int PositionPattern::getNumOfTimePatternPoint() const
{
	return numOfTimePatternPoint;
}
void PositionPattern::addNumOfTimePatternPoint()
{
	++numOfTimePatternPoint;
}

int PositionPattern::getSize() const
{
	return corners.size();
}

bool PositionPattern::isEmpty() const
{
	return corners.empty();
}

Point2f PositionPattern::operator[] (int x) const
{
	return corners[x];
}


vector<int>& PositionPattern::getTimePatternSidePoint()
{
	return timePatternSidePoint;
}
int PositionPattern::getTimePatternPoint()
{
	return timePatternPoint;
}




Vec2f twoPt2vec2f(cv::Point pt1, Point pt2)
{
	if (pt1 != pt2 || pt1 != Point(0, 0) || pt2 != Point(0, 0))
	{
		int y = abs(pt1.y - pt2.y);
		int x = abs(pt1.x - pt2.x);
		double theta = atan(static_cast<double>(x) / static_cast<double>(y));
		if (pt1.y >= pt2.y)
		{
			if (pt1.x >= pt2.x)
				theta = CV_PI - theta;
			else
				theta = -CV_PI + theta;
		}
		else
		{
			if (pt1.x < pt2.x)
				theta = CV_PI - theta;
			else
				theta = -CV_PI + theta;
		}

		double pt1ToO, pt2ToO, pt1ToPt2;
		pt1ToO = sqrt(pt1.x*pt1.x + pt1.y*pt1.y);
		pt2ToO = sqrt(pt2.x*pt2.x + pt2.y*pt2.y);
		pt1ToPt2 = sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.y)*(pt2.y - pt1.y));
		double angleOf2Line = acos((pt1ToPt2*pt1ToPt2 + pt2ToO * pt2ToO - pt1ToO * pt1ToO) / (2 * pt1ToPt2*pt2ToO));
		double verDist = pt2ToO * sin(angleOf2Line);

		if (fabs(theta) < CV_PI / 4. || fabs(theta) > 3.*CV_PI / 4.)
		{
			verDist = -verDist;
		}
		if (fabs(fabs(theta) - CV_PI) < DBL_EPSILON)
		{
			theta = 0;
			verDist = fabs(verDist);
		}

		Vec2f output(verDist, theta);
		return output;
	}
	else
	{
		Vec2f output(0, 0);
		return output;
	}
}

vector<Point2f> lineToPointPair(Vec2f line)
{
	vector<Point2f> points;

	float r = line[0], t = line[1];
	double cos_t = cos(t), sin_t = sin(t);
	double x0 = r * cos_t, y0 = r * sin_t;
	double alpha = 1000;

	points.push_back(Point2f(x0 + alpha * (-sin_t), y0 + alpha * cos_t));
	points.push_back(Point2f(x0 - alpha * (-sin_t), y0 - alpha * cos_t));

	return points;
}

Point2f computeIntersect(const Vec2f line1, const Vec2f line2)
{
	vector<Point2f> p1 = lineToPointPair(line1);
	vector<Point2f> p2 = lineToPointPair(line2);

	float denom = (p1[0].x - p1[1].x)*(p2[0].y - p2[1].y) - (p1[0].y - p1[1].y)*(p2[0].x - p2[1].x);
	Point2f intersect(((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].x - p2[1].x) -
		(p1[0].x - p1[1].x)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom,
		((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].y - p2[1].y) -
		(p1[0].y - p1[1].y)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom);

	return intersect;
}

Point2f computeIntersect(const Vec4f &line1, const Vec4f &line2)
{
	float x, y;
	float x1 = line1[2];
	float y1 = line1[3];
	float x2 = line2[2];
	float y2 = line2[3];

	if (fabs(line1[0]) <= FLT_EPSILON || fabs(line2[0]) <= FLT_EPSILON)
	{
		if (fabs(line1[0]) <= FLT_EPSILON && fabs(line2[0]) <= FLT_EPSILON)
		{
			return Point2f(0, 0);
		}
		else if (fabs(line1[0]) <= FLT_EPSILON)
		{
			float m2 = line2[1] / line2[0];
			x = x1;
			y = m2 * (x - x2) + y2;

		}
		else
		{
			float m1 = line1[1] / line1[0];
			x = x2;
			y = m1 * (x - x1) + y1;
		}

	}
	else
	{
		float m1 = line1[1] / line1[0];
		float m2 = line2[1] / line2[0];
		if (m1 < m2 + FLT_EPSILON && m1 > m2 - FLT_EPSILON)
		{
			return Point2f(0, 0);
		}
		x = (m2*x2 - y2 - m1 * x1 + y1) / (m2 - m1);
		y = m1 * (x - x1) + y1;
	}

	if (x > 0 && y > 0)
	{
		return Point2f(x, y);
	}
	else
	{
		return Point2f(0, 0);
	}
}

inline float computeDistance(const Point &i, const Point &j)
{
	return sqrt((j.y - i.y)*(j.y - i.y) + (j.x - i.x)*((j.x - i.x)));
}
inline float computeDistance(const Point2f &i, const Point2f &j)
{
	return sqrt((j.y - i.y)*(j.y - i.y) + (j.x - i.x)*((j.x - i.x)));
}

bool DescendingSize(vector<Point> &i, vector<Point> &j)
{
	return (i.size() > j.size());
}

bool DescendingSlope(Vec4f &i, Vec4f &j)
{
	if (fabs(i[0]) <= FLT_EPSILON && fabs(j[0]) <= FLT_EPSILON)
		return i[1] > 0;
	else if (fabs(i[0]) <= FLT_EPSILON)
		return i[1] > 0;
	else if (fabs(j[0]) <= FLT_EPSILON)
		return j[1] < 0;

	return fabs(i[1] / i[0]) > fabs(j[1] / j[0]);
}

/* don't check empty */
/* assume same index */
bool DescendingFistVal(vector<int> &i, vector<int> &j)
{
	if (i[0] == j[1])
		swap(j[0], j[1]);
	if (i[1] == j[0])
		swap(i[0], i[1]);

	return (*i.begin() > *j.begin());
}



float computeMean(const std::vector<int> &numbers)
{
	if (numbers.empty())
		return 0;

	float total = 0;
	for (int number : numbers) {
		total += number;
	}
	//cout << "mean: " << total / numbers.size() << endl;
	return (total / numbers.size());
}

float computeVariance(const std::vector<int> &numbers)
{
	float mean = computeMean(numbers);

	float result = 0;
	for (int number : numbers)
	{
		result += (number - mean)*(number - mean);
	}
	//cout << "var: " << result / (numbers.size() - 1) << endl;
	return result / (numbers.size() - 1);
}
