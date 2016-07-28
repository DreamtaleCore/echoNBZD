#include "opencvHeaders.h"
#include "stdHeader.h"

const int kvalue = 15;

int main()
{
	Mat src_color = imread("G:\\1\\round1.jpg");
	imshow("Src", src_color);

	Mat src_gray;
	cvtColor(src_color, src_gray, COLOR_BGR2GRAY);
	imshow("Gray", src_gray);

	Mat bf;
	bilateralFilter(src_gray, bf, kvalue, kvalue * 2, kvalue / 2);
	imshow("Filter", bf);

	Mat dst = src_color;

	vector<Vec3f> circles;
	HoughCircles(bf, circles, CV_HOUGH_GRADIENT, 1.5, 20, 130, 38, 10, 50);

	cout << "x=\ty=\tr=\t" << endl;
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		circle(dst, center, 0, Scalar(0, 255, 255), -1, 8, 0);
		circle(dst, center, radius, Scalar(255, 0, 0), -1, 1, 0);

		cout << cvRound(circles[i][0]) << "\t"
			<< cvRound(circles[i][1]) << "\t"
			<< cvRound(circles[i][2]) << endl;
	}

	imshow("Dst", dst);

	while (true)
	{
		char c = waitKey(30);
		if (c == 27)
			break;
	}

	return 0;
}