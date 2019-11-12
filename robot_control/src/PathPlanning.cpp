
#include "PathPlanning.h"

PathPlanning::PathPlanning()
{
}

void PathPlanning::mapIntoCells()
{
    cv::Mat image;
    image = cv::imread("../testImages/SmallWorldV2.png", cv::IMREAD_COLOR);
    int resize = 30;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows));

    int GRID_SIZE = 0.50 * resize;

	int width = image.cols;
	int height = image.rows;
	std::vector<cv::Rect> mCells;

	for (int y = 0; y < height - GRID_SIZE + 1; y += GRID_SIZE) 
	{
		for (int x = 0; x < width - GRID_SIZE + 1; x += GRID_SIZE) 
		{
			int k = x * y + x;
			cv::Rect grid_rect(x, y, GRID_SIZE, GRID_SIZE);
			//cout << grid_rect << endl;
			mCells.push_back(grid_rect);
			cv::rectangle(image, grid_rect, cv::Scalar(0, 255, 0), 1);
			cv::namedWindow("Grid", CV_WINDOW_AUTOSIZE); // Create a window
			cv::imshow("Grid", image);
			//imshow(format("grid%d", k), mapResized(grid_rect));
		}
	}
}

cv::Mat PathPlanning::wavefront()
{
    cv::Mat image;
    image = cv::imread("../testImages/SmallWorldV2.png", cv::IMREAD_COLOR);
    int resize = 30;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows));
    //cv::cvtColor(image, image, CV_BGR2GRAY);

    int GRID_SIZE = 0.50 * resize;

	int width = image.cols;
	int height = image.rows;
	std::vector<cv::Rect> mCells;

    //cv::Mat cells;
    std::vector<cellValue> cellVector;
    int count = 0;
    bool check = false;
    int gridCnt = 0;

    for (int y = 0; y < height - GRID_SIZE + 1; y += GRID_SIZE) 
	{
		for (int x = 0; x < width - GRID_SIZE + 1; x += GRID_SIZE) 
		{
            gridCnt++;

            struct cellValue c;
            c.p1 = cv::Point2f(x, y);
            c.p2 = cv::Point2f(x + GRID_SIZE, y + GRID_SIZE);

            for (int i = c.p1.x; i <= c.p2.x; i++)
            {
                for (int j = c.p1.y; j <= c.p2.y; j++)
                {
                    cv::Vec3b point = image.at<cv::Vec3b>(j, i);

                    // Give the value 1 to obstacles
                    if (point[0] < 200)
                    {
                        c.value = 1;

                        if (check == false)
                        {
                            count++;
                            check = true;
                        }
                    }
                }
            }
            cellVector.push_back(c);
            std::cout << "p1: " << "(" << cellVector[0].p1.x << "," << cellVector[0].p1.y << ")" << std::endl;
            std::cout << "p2: " << "(" << cellVector[0].p2.x << "," << cellVector[0].p2.y << ")" << std::endl;
            std::cout << "value: " << cellVector[0].value << std::endl;
            check = false;
		}
	}

    std::cout << count << std::endl;
    std::cout << "Grid: " << gridCnt << std::endl;
    return image;
}

PathPlanning::~PathPlanning()
{
}