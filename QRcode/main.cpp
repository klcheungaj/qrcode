#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/cuda.hpp"
#include <vector>
#include <math.h>
#include <chrono> 
#include "PositionPattern.h"
//#include "zbar.h"

using namespace cv;
using namespace std;
//using namespace zbar;
int thresh = 50;
RNG rng(12345);
int erodeVvalue;
#define	SIZE_LIMIT	800

Mat drawingCont(cv::Size size, std::vector<vector<Point> > &contours, std::vector<Vec4i> &hierarchy)
{
	Mat drawCont = Mat::zeros(size, CV_8UC1);
	for (int i = 0; i < contours.size(); i++)
	{
		drawContours(drawCont, contours, i, (255, 255, 255), 1, LINE_AA, hierarchy, 0, Point());
	}
	imshow("contours"+to_string(contours.size()), drawCont);

	return drawCont;
}

int main(int argc, char const *argv[]) {
	/*auto start = std::chrono::steady_clock::now();*/
	RNG rng;
	/* read image */
	Mat image = imread("test4.jpg");
	if (!image.data)
	{
		//cout << "confirm picture" << endl;
		system("pause");
		return 0;
	}
	if (image.cols > 800)
	{
		if (image.cols > image.rows)
			resize(image, image, Size(800, static_cast<float>(image.rows) / image.cols * 800), 0, 0, INTER_LINEAR);
		else
			resize(image, image, Size(static_cast<float>(image.cols) / image.rows * 800, 800), 0, 0, INTER_LINEAR);
	}
	else if (image.rows > 800)
	{
		if (image.cols > image.rows)
			resize(image, image, Size(800, static_cast<float>(image.rows) / image.cols * 800), 0, 0, INTER_LINEAR);
		else
			resize(image, image, Size(static_cast<float>(image.cols) / image.rows * 800, 800), 0, 0, INTER_LINEAR);
	}
	else if (image.cols < 800)
	{
		if (image.cols > image.rows)
			resize(image, image, Size(800, static_cast<float>(image.rows) / image.cols * 800), 0, 0, INTER_LINEAR);
		else
			resize(image, image, Size(static_cast<float>(image.cols) / image.rows * 800, 800), 0, 0, INTER_LINEAR);
	}
	else if (image.rows < 800)
	{
		if (image.cols > image.rows)
			resize(image, image, Size(800, static_cast<float>(image.rows) / image.cols * 800), 0, 0, INTER_LINEAR);
		else
			resize(image, image, Size(static_cast<float>(image.cols) / image.rows * 800, 800), 0, 0, INTER_LINEAR);
	}

	Mat imageGray;
	cvtColor(image, imageGray, CV_RGB2GRAY);
	//imshow("gray", imageGray);

	/* threshold */
	Mat adapThresh = Mat::zeros(imageGray.size(), CV_8UC1);
	//threshold(imageGray, adapThresh, 130, 255, THRESH_BINARY);
	//threshold(imageGray, adapThresh, 70, 255, THRESH_OTSU);
	adaptiveThreshold(imageGray, adapThresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 25, 0);
	imshow("adaptive threshold", adapThresh);

	/* find contours */
	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;
	//Mat canny_output;
	//Canny(imageGray, canny_output, thresh, thresh * 2, 3);
	findContours(adapThresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	//drawingCont(imageGray.size(), contours, hierarchy);
	Mat drawing = Mat::zeros(imageGray.size(), CV_8UC1);

	/* find contours with 3 layer*/
	vector<vector<Point> > contours_filter;
	vector<int> contours_index;
	for (int i = 0; i < contours.size(); i++)
	{
		int k = i;
		int numOfHier = 0;
		while (hierarchy[k][2] != -1) {
			k = hierarchy[k][2];
			++numOfHier;
		}

		if (numOfHier >= 2)
		{
			contours_filter.push_back(contours[i]);
			contours_index.push_back(i);
			//cout << i << " contour size: " << contours[i].size() << endl;
			drawContours(drawing, contours, i, (255, 255, 255), 1, LINE_AA, hierarchy, 0, Point());
		}
	}
	drawing = drawingCont(imageGray.size(), contours_filter, hierarchy).clone();

	/* find corners */
	vector<PositionPattern*>posPat;
	posPat.reserve(contours_filter.size());
	int coutour_count = 0;
	for (int k=0 ; k<contours_filter.size() ; k++)
	{
		posPat.push_back(new PositionPattern(contours_filter[k], (arcLength(contours_filter[k], true) - arcLength(contours[hierarchy[contours_index[k]][2]], true))/8));
		if ((*(posPat.end() - 1))->isEmpty())
		{
			delete (*(posPat.end() - 1));
			posPat.pop_back();
		}
		else
		{
			++coutour_count;
		}
	}

	if (posPat.empty())
	return 0;

	for (PositionPattern* p_posPat : posPat)
	{
		if (p_posPat)
		{
			for (int i = 0; i < p_posPat->getSize(); i++)
			{
				circle(drawing, (*p_posPat)[i], 3, (255, 255, 255), 2);
			}
		}
	}
	imshow("corners", drawing);

	Mat patternImg = drawing.clone();

	/* find time pattern */
	vector<vector<int>> QRCode_group;
	for (int i = 0; i < posPat.size() - 1; i++)
	{
		Mat drawing_time_pattern = adapThresh.clone();
		for (int j = i + 1; j < posPat.size(); j++)
		{
			Vec4i timePattern = posPat[i]->detectTimePattern(*(posPat[j]), adapThresh, drawing_time_pattern, 5);
			//line(drawing_time_pattern, Point(timePattern[0], timePattern[1]), Point(timePattern[2], timePattern[3]), Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3, 8, 0);

			if (timePattern[0] != 0 && timePattern[1] != 0 && timePattern[2] != 0 && timePattern[3] != 0)
			{
				line(patternImg, Point(timePattern[0], timePattern[1]), Point(timePattern[2], timePattern[3]), Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3, 8, 0);

				if (QRCode_group.empty())
				{
					QRCode_group.push_back(vector<int>());
					QRCode_group[0].push_back(i);
					QRCode_group[0].push_back(j);
				}
				else
				{
					bool findQRCode = false;
					for (vector<int>& QRCode : QRCode_group)
					{
						if (!findQRCode)
						{
							if (find(QRCode.begin(), QRCode.end(), i) != QRCode.end())
							{
								QRCode.push_back(j);
								findQRCode = true;
							}
							else if (find(QRCode.begin(), QRCode.end(), j) != QRCode.end())
							{
								QRCode.push_back(i);
								findQRCode = true;
							}
						}
					}
					if (!findQRCode)
					{
						QRCode_group.push_back(vector<int>());
						QRCode_group[QRCode_group.size() - 1].push_back(i);
						QRCode_group[QRCode_group.size() - 1].push_back(j);
					}
				}
			}
		}
		imshow(to_string(i), drawing_time_pattern);
	}
	imshow("time pattern", patternImg);

	cout << "QRCode group size: " << QRCode_group.size() << endl;
	if (!QRCode_group.empty())
	{
		for (vector<int> QRCode : QRCode_group)
		{
			cout << "QRCODE SIZE: " << QRCode.size() <<endl;
			if (QRCode.size() == 3)
			{
				vector<Vec4f> findIntersect;
				findIntersect.reserve(2);
				vector<Point2f> boundingCorners;
				for (int index : QRCode)
				{
					Vec4f temp;
					if (posPat[index]->getBoundingLine(temp))
					{
						findIntersect.push_back(temp);
					}
					boundingCorners.push_back(posPat[index]->getBoundingPoint());
				}
				cout << "corners size: " << boundingCorners.size() << endl;
				cout << "findIntersect size: " << findIntersect.size() << endl;

				if (findIntersect.size() == 2)
				{
					boundingCorners.push_back(computeIntersect(findIntersect[0], findIntersect[1]) );
				}

				cout << "corners size: " << boundingCorners.size() <<endl;

				if (boundingCorners.size() == 4)
				{
					Mat drawing_corners = imageGray.clone();
					for (int k = 0; k < 4; k++)
					{

						circle(drawing_corners, boundingCorners[k], 3, (255, 255, 255), 2);
					}
					imshow("drawing corners", drawing_corners);

					/* sort */
					for (int i = 0; i < 4; i++)
					{
						if (boundingCorners[i].y < boundingCorners[0].y)
							swap(boundingCorners[0], boundingCorners[i]);
					}
					for (int j = 1; j < 4; j++)
					{
						if (boundingCorners[j].y < boundingCorners[1].y)
							swap(boundingCorners[1], boundingCorners[j]);
					}
					if (boundingCorners[1].x < boundingCorners[0].x)
						swap(boundingCorners[1], boundingCorners[0]);
					if (boundingCorners[3].x > boundingCorners[2].x)
						swap(boundingCorners[3], boundingCorners[2]);

					Point2f srcPoints[4] = { boundingCorners[0], boundingCorners[1], boundingCorners[2], boundingCorners[3] };

					int width = posPat[QRCode[0]]->getWidth();
					srcPoints[0].x -= width;
					srcPoints[0].y -= width;
					srcPoints[1].x += width;
					srcPoints[1].y -= width;
					srcPoints[2].x += width;
					srcPoints[2].y += width;
					srcPoints[3].x -= width;
					srcPoints[3].y += width;
					for (int m = 0; m < 4 ; m++)
					{
						if (srcPoints[m].x < 0)
							srcPoints[m].x = 0;
						if (srcPoints[m].y < 0)
							srcPoints[m].y = 0;
					}


					/* transform */
					Point2f dstPoints[4];
					Mat outPic = Mat::zeros(Size(300, 300), CV_8UC3);
					dstPoints[0] = Point2f(0, 0);
					dstPoints[1] = Point2f(outPic.cols, 0);
					dstPoints[2] = Point2f(outPic.cols, outPic.rows);
					dstPoints[3] = Point2f(0, outPic.rows);

					Mat transMat = getPerspectiveTransform(srcPoints, dstPoints);
					warpPerspective(imageGray, outPic, transMat, outPic.size(), INTER_LINEAR);
					imshow("transform", outPic);
					cout << outPic.size();

					imwrite("./after_transform.jpg", outPic);
				}
			}
		}
	}





	///* detection QR Code */
	//ImageScanner scanner;
	//scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
	//int width = imageGray.cols;
	//int height = imageGray.rows;
	//uchar *raw = (uchar *)imageGray.data;
	//Image imageZbar(width, height, "Y800", raw, width * height);
	//scanner.scan(imageZbar);     
	//Image::SymbolIterator symbol = imageZbar.symbol_begin();
	//if (imageZbar.symbol_begin() == imageZbar.symbol_end())
	//{
	//	//cout << "detection failed" << endl;
	//}
	//for (; symbol != imageZbar.symbol_end(); ++symbol)
	//{
	//	cout << "type：" << endl << symbol->get_type_name() << endl << endl;
	//	cout << "data：" << endl << symbol->get_data() << endl << endl;
	//}
	////imshow("Source Image", image);
	//imageZbar.set_data(NULL, 0);

	/*auto end = std::chrono::steady_clock::now();
	std::cout << endl << "that took " << std::chrono::duration_cast<chrono::milliseconds>(end - start).count()
		<< " milliseconds\n";*/

	waitKey();
}