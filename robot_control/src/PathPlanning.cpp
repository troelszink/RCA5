
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

std::vector<cellValue> PathPlanning::wavefront()
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
                    if (point[0] < 240)
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
            //std::cout << "p1: " << "(" << cellVector[0].p1.x << "," << cellVector[0].p1.y << ")" << std::endl;
            //std::cout << "p2: " << "(" << cellVector[0].p2.x << "," << cellVector[0].p2.y << ")" << std::endl;
            //std::cout << "value: " << cellVector[0].value << std::endl;
            check = false;
		}
	}

    std::cout << count << std::endl;
    std::cout << "Grid: " << gridCnt << std::endl;

    cv::Point2f center = cv::Point2f(10, 7.5);
    float scaling = 1.41735;

    // Start point
    float xPixel = start.x * scaling + center.x;
    float yPixel = -start.y * scaling + center.y;
    int cellIndex = int(yPixel/0.5) * 40 + int(xPixel/0.5);
    if (cellVector[cellIndex].value != 1)
    {
        cellVector[cellIndex].value = 0;
    }
    else
    {
        std::cout << "Not a valid start." << std::endl;
    }

    // Goal point
    xPixel = goal.x * scaling + center.x;
    yPixel = -goal.y * scaling + center.y;
    cellIndex = int(yPixel/0.5) * 40 + int(xPixel/0.5);
    if (cellVector[cellIndex].value != 1)
    {
        cellVector[cellIndex].value = 2;
    }
    else
    {
        std::cout << "Not a valid goal." << std::endl;
    }
    std::cout << xPixel/0.5 << " " << yPixel/0.5 << " " << cellIndex << std::endl;

    int cellsFilled = 0;
    int i = cellIndex - 1; // Starting to the left of the goal position
    int j = 0;
    int cellsNotFilled = 726;
    int lowestNeighbor = 1000;
    int up = 40;
    int right = 1;
    int down = 40;
    int left = 1;

    while (cellsFilled < cellsNotFilled)
    {
        /*j++;
        if (j == 99999)
        {
            std::cout << "CellsNotFilled" << cellsNotFilled << std::endl; 
            std::cout << "CellsFilled" << cellsFilled << std::endl;
            std::cout << "Stopped" << std::endl;
        }*/
        lowestNeighbor = 1000;

        if (cellVector[i].value < 0)
        {
            /*if (i - 41 > 0) // upper-left neighbor
            {
                if (cellVector[i - 41].value > 1)
                {
                    lowestNeighbor = cellVector[i - 41].value;
                }
            }*/
            if (i - up > 0) // upper neighbor
            {
                if (cellVector[i - up].value > 1 && cellVector[i - up].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i - up].value;
                }
            }
            /*if (i - 39 > 0) // upper-right neighbor
            {
                if (cellVector[i - 39].value > 1 && cellVector[i - 39].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i - 39].value;
                }
            }*/
            if (i + right < cellVector.size()) // right neighbor
            {
                if (cellVector[i + right].value > 1 && cellVector[i + right].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i + right].value;
                }
            }
            /*if (i + 41 < cellVector.size()) // lower-right neighbor
            {
                if (cellVector[i + 41].value > 1 && cellVector[i + 41].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i + 41].value;
                }
            }*/
            if (i + down < cellVector.size()) // lower neighbor
            {
                if (cellVector[i + down].value > 1 && cellVector[i + down].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i + down].value;
                }
            }
            /*if (i + 39 < cellVector.size()) // lower-left neighbor
            {
                if (cellVector[i + 39].value > 1 && cellVector[i + 39].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i + 39].value;
                }
            }*/
            if (i - left > 0) // left neighbor
            {
                if (cellVector[i - left].value > 1 && cellVector[i - left].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i - left].value;
                }
            }
        }

        // Setting the cell value for the specified cell
        if (lowestNeighbor < 1000)
        {
            cellVector[i].value = lowestNeighbor + 1;
            cellsFilled++;
        }
        
        i++;
        if (i == cellVector.size())
        {
            i = 0;
        }
    }

    for (int i = 0; i < cellVector.size(); i++)
    {
        std::cout << cellVector[i].value << " ";
        if (cellVector[i].value < 10 && cellVector[i].value > -1)
        {
            std::cout << " ";
        }   
        if ((i + 1) % 40 == 0)
        {
            std::cout << std::endl;
        }
    }
    
    return cellVector;
}

cv::Point2f PathPlanning::robotControl(std::vector<cellValue> cellVector, cv::Point2f currentPos)
{
    cv::Point2f wayPoint;

    cv::Point2f center = cv::Point2f(10, 7.5);
    float scaling = 1.41735;
    
    // Start point
    float xPixel = currentPos.x * scaling + center.x;
    float yPixel = -currentPos.y * scaling + center.y;
    int cellIndex = int(yPixel/0.5) * 40 + int(xPixel/0.5);

    int i = cellIndex; // Starting to the left of the goal position
    int lowestNeighborValue = 1000;
    int lowestNeighborIndex = 0;
    int up = 40;
    int right = 1;
    int down = 40;
    int left = 1;
   
            if (i - up > 0) // upper neighbor
            {
                if (cellVector[i - up].value > 1 && cellVector[i - up].value < lowestNeighborValue)
                {
                    lowestNeighborIndex = i - up;
                    lowestNeighborValue = cellVector[i - up].value;
                }
            }

            if (i + right < cellVector.size()) // right neighbor
            {
                if (cellVector[i + right].value > 1 && cellVector[i + right].value < lowestNeighborValue)
                {
                    lowestNeighborIndex = i + right;
                    lowestNeighborValue = cellVector[i + right].value;
                }
            }

            if (i + down < cellVector.size()) // lower neighbor
            {
                if (cellVector[i + down].value > 1 && cellVector[i + down].value < lowestNeighborValue)
                {
                    lowestNeighborIndex = i + down;
                    lowestNeighborValue = cellVector[i + down].value;
                }
            }

            if (i - left > 0) // left neighbor
            {
                if (cellVector[i - left].value > 1 && cellVector[i - left].value < lowestNeighborValue)
                {
                    lowestNeighborIndex = i - left;
                    lowestNeighborValue = cellVector[i - left].value;
                }
            }

    wayPoint.x = ((cellVector[lowestNeighborIndex].p1.x + (cellVector[lowestNeighborIndex].p2.x - cellVector[lowestNeighborIndex].p1.x)/2) / 30 - center.x) / scaling;
    wayPoint.y = (-(cellVector[lowestNeighborIndex].p1.y + (cellVector[lowestNeighborIndex].p2.y - cellVector[lowestNeighborIndex].p1.y)/2) / 30 + center.y) / scaling;

    std::cout << "P1: " << cellVector[lowestNeighborIndex].p1.x << "," << cellVector[lowestNeighborIndex].p1.y << std::endl;
    std::cout << "P2: " << cellVector[lowestNeighborIndex].p2.x << "," << cellVector[lowestNeighborIndex].p2.y << std::endl;
    std::cout << "xPixel: " << xPixel << " " << "yPixel: " << yPixel << " " << "CellIndex: " << cellIndex << std::endl;
    std::cout << "CurrentPos: " << currentPos << std::endl;
    std::cout << "Lowest Neighbor: " << lowestNeighborIndex << std::endl;
    std::cout << "wayPoint: " << wayPoint.x << "," << wayPoint.y << std::endl << std::endl;

    return wayPoint;
}

cv::Point2f PathPlanning::getGoal()
{
    return goal;
}

PathPlanning::~PathPlanning()
{
}