#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp> 

using namespace cv;
class Histogram1D
{
private:
	int histSize[3];
	float hranges[2];
	const float* ranges[3];
	int channels[3];
public:
	Histogram1D()
	{
		histSize[0] = histSize[1] = histSize[2] = 256;
		hranges[0] = 0.0;
		hranges[1] = 255.0;
		ranges[0] = hranges;
		ranges[1] = hranges;
		ranges[2] = hranges;
		channels[0] = 0;
		channels[1] = 1;
		channels[2] = 2;
	}
	MatND getHistogram(const Mat &img)
	{
		MatND hist;
		calcHist(&img, 1, channels, Mat(), hist, 3, histSize, ranges);
	}
	Mat getHistogramImage(const Mat &image)
	{
		MatND hist;
		calcHist(&image, 1, channels, Mat(), hist, 1, histSize, ranges);
		double maxVal = 0;
		double minVal = 0;
		minMaxLoc(hist, &minVal, &maxVal, 0, 0);
		Mat histImg(histSize[0], histSize[0], CV_8U, Scalar(255));
		int hpt = static_cast<int>(0.9*histSize[0]);
		for (int h = 0; h < histSize[0]; h++)
		{
			float binVal = hist.at<float>(h);
			int intensity = static_cast<int>(binVal*hpt / maxVal);
			line(histImg, Point(h, histSize[0]), Point(h, histSize[0] - intensity), Scalar::all(0));
		}
		return histImg;
	}
};