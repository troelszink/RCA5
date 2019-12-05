
#include "PathPlanning.h"

PathPlanning::PathPlanning()
{
}

// Used to draw the cells that we divide the environment into. Just to visualize, not necessary for our project to work
void PathPlanning::mapIntoCells()
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);
    int resize = 10;
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

// Implementing the Wavefront planner to go from point A to point B
std::vector<cellValueWF> PathPlanning::wavefront()
{
    cv::Mat image;
    image = cv::imread("../testImages/SmallWorldV2.png", cv::IMREAD_COLOR);
    int resize = 30;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows));

    int GRID_SIZE = 0.50 * resize;

	int width = image.cols;
	int height = image.rows;
	std::vector<cv::Rect> mCells;

    std::vector<cellValueWF> cellVector;
    int count = 0;
    bool check = false;
    int gridCnt = 0;

    // Drawing the environment with values of '1' at the walls and '-1' at the rest (-1 is defined inside the .h-file)
    for (int y = 0; y < height - GRID_SIZE + 1; y += GRID_SIZE) 
	{
		for (int x = 0; x < width - GRID_SIZE + 1; x += GRID_SIZE) 
		{
            gridCnt++;

            struct cellValueWF c;
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
            check = false;
		}
	}

    //std::cout << count << std::endl;
    //std::cout << "Grid: " << gridCnt << std::endl;

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

    // Different values needed for the next part
    int cellsFilled = 0;
    int i = cellIndex - 1; // Starting to the left of the goal position
    int j = 0;
    int cellsNotFilled = 726;
    int lowestNeighbor = 1000;
    int up = 40;
    int right = 1;
    int down = 40;
    int left = 1;

    // As long as there are still cells that are not filled with a value different from '-1', keep going
    while (cellsFilled < cellsNotFilled)
    {
        lowestNeighbor = 1000;

	// Checking if the current value is an undefined valuye, '-1'
        if (cellVector[i].value < 0)
        {
	    // Using the 4-adjadency (upper, right, lower, left) to insert values into the Wavefront planner
            if (i - up > 0) // upper neighbor
            {
                if (cellVector[i - up].value > 1 && cellVector[i - up].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i - up].value;
                }
            }
            if (i + right < cellVector.size()) // right neighbor
            {
                if (cellVector[i + right].value > 1 && cellVector[i + right].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i + right].value;
                }
            }
            if (i + down < cellVector.size()) // lower neighbor
            {
                if (cellVector[i + down].value > 1 && cellVector[i + down].value < lowestNeighbor)
                {
                    lowestNeighbor = cellVector[i + down].value;
                }
            }
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

    // Prints the Wavefront planner out to the console - Mostly for testing
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

// Determines the path to go for the robot
cv::Point2f PathPlanning::robotControl(std::vector<cellValueWF> cellVector, cv::Point2f currentPos)
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
   
	    // Checks if the robot has to go up, right, down or left to obtain the shortest path (or one of them at least)
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

    // Saving waypoints for the robot to follow (using the Fuzzy Controller)
    wayPoint.x = ((cellVector[lowestNeighborIndex].p1.x + (cellVector[lowestNeighborIndex].p2.x - cellVector[lowestNeighborIndex].p1.x)/2) / 30 - center.x) / scaling;
    wayPoint.y = (-(cellVector[lowestNeighborIndex].p1.y + (cellVector[lowestNeighborIndex].p2.y - cellVector[lowestNeighborIndex].p1.y)/2) / 30 + center.y) / scaling;

    return wayPoint;
}

cv::Point2f PathPlanning::getGoal()
{
    return goal;
}

// Implementation of the Brushfire Algorithm together with the GVD (General Voronoi Diagram)
std::vector<cellValueBF> PathPlanning::brushfire()
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);
    int resize = 1;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows));

    float GRID_SIZE = 0.50 * resize;

	int width = image.cols;
	int height = image.rows;
	std::vector<cv::Rect> mCells;

    std::vector<cellValueBF> cellVector;
    int count = 0;
    bool check = false;
    int gridCnt = 0;

    // Drawing the environment with values of '1' at the walls and '-1' at the rest (-1 is defined inside the .h-file)
    // Just as we did for the Wavefront planner
    for (float y = 0; y < height; y += GRID_SIZE) 
	{
		for (float x = 0; x < width; x += GRID_SIZE) 
		{
            gridCnt++;

            struct cellValueBF c;
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
            check = false;
		}
	}

    //std::cout << count << std::endl;
    //std::cout << "Grid: " << gridCnt << std::endl;

    cv::Point2f center = cv::Point2f(60, 40);
    float scaling = 1.41735;

    // Different values needed for the next part
    int cellsFilled = 0;
    int i = 0;
    int j = 0;
    int cellsNotFilled = resize * 240 * 160 - count;
    int up = 240;
    int right = 1;
    int down = 240;
    int left = 1;
    int cellCheck = 1;

    // As long as there are still cells that are not filled with a value different from '-1', keep going
    while (cellsFilled < cellsNotFilled)
    {
        if (cellVector[i].value == cellCheck)
        {
            if (i - up > 0) // upper neighbor
            {
                if (cellVector[i - up].value == 0)
                {
                    cellVector[i - up].value = cellCheck + 1;
                    cellsFilled++;                   
                }
            }
            if (i + right < cellVector.size()) // right neighbor
            {
                if (cellVector[i + right].value == 0)
                {
                    cellVector[i + right].value = cellCheck + 1;
                    cellsFilled++;                   
                }
            }
            if (i + down < cellVector.size()) // lower neighbor
            {
                if (cellVector[i + down].value == 0)
                {
                    cellVector[i + down].value = cellCheck + 1;
                    cellsFilled++;                
                }
            }
            if (i - left > 0) // left neighbor
            {
                if (cellVector[i - left].value == 0)
                {
                    cellVector[i - left].value = cellCheck + 1;
                    cellsFilled++;
                }
            }
        }
        
        i++;

        if (i == cellVector.size())
        {
            i = 0;
            cellCheck++;
        }
    }

    std::vector<cellValueBF> maxCell;

    // Drawing the Brushfire algorithm and the GVD
    for (int i = 0; i < cellVector.size(); i++)
    {
        if (cellVector[i].value == 1)
        {
            continue;
        }
        if (i - up > 0) // upper neighbor
        {
            if (cellVector[i - up].value > cellVector[i].value)
            {
                continue;                 
            }
        }
        if (i + right < cellVector.size()) // right neighbor
        {
            if (cellVector[i + right].value > cellVector[i].value)
            {
                continue;                   
            }
        }
        if (i + down < cellVector.size()) // lower neighbor
        {
            if (cellVector[i + down].value > cellVector[i].value)
            {
                continue;                 
            }
        }
        if (i - left > 0) // left neighbor
        {
            if (cellVector[i - left].value > cellVector[i].value)
            {
                continue;
            }
        }

        if (removeCorners(i, cellVector) && isValidRoom(i, cellVector))
        {
            maxCell.push_back(cellVector[i]);

            for (float f = cellVector[i].p1.x; f < cellVector[i].p2.x; f++)
            {
                for (float s = cellVector[i].p1.y; s < cellVector[i].p2.y; s++)
                {
                    image.at<cv::Vec3b>(s, f)[0] = 255;
                    image.at<cv::Vec3b>(s, f)[1] = 0;
                    image.at<cv::Vec3b>(s, f)[2] = 0;
                }
            }
        }
        else if (isValidRoom(i, cellVector))
        {
            for (float f = cellVector[i].p1.x; f < cellVector[i].p2.x; f++)
            {
                for (float s = cellVector[i].p1.y; s < cellVector[i].p2.y; s++)
                {
                    image.at<cv::Vec3b>(s, f)[0] = 0;
                    image.at<cv::Vec3b>(s, f)[1] = 0;
                    image.at<cv::Vec3b>(s, f)[2] = 255;
                }
            }
        }
        
    }

    resize = 10;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, cv::INTER_NEAREST);
    cv::namedWindow("Brushfire", CV_WINDOW_AUTOSIZE);
    cv::imshow("Brushfire", image);
    cv::imwrite( "../testImages/BigWorldV2-BF1.png", image );
    cv::waitKey();
    
    return cellVector;
}

// Removes the GVD from the corners (the red cells described in the report, only keep the blue cells)
bool PathPlanning::removeCorners(int index, std::vector<cellValueBF> cellVector)
{
    int up = 240;
    int right = 1;
    int down = 240;
    int left = 1;
    int i = index;
    int countUp = 0;
    int countRight = 0;
    int countDown = 0;
    int countLeft = 0;

    while(true)
    {
        while (true)
        {
            countUp++;
            if (i - up < 0) // upper neighbor
            {
                return false;
            } 
            if (cellVector[i - up].value == 1)
            {
                break;
            }
            i = i - up;
        }

        i = index;
        while (true)
        {
            countRight++;
            if ((i + right) % 240 == 0) // right neighbor
            {
                return false;
            } 
            if (cellVector[i + right].value == 1)
            {
                break;
            }
            i = i + right;
        }

        i = index;
        while (true)
        {
            countDown++;
            if (i + down >= cellVector.size()) // down neighbor
            {
                return false;
            } 
            if (cellVector[i + down].value == 1)
            {
                break;
            }
            i = i + down;
        }

        i = index;
        while (true)
        {
            countLeft++;
            if ((i - left) % 240 == 239) // left neighbor
            {
                return false;
            } 
            if (cellVector[i - left].value == 1)
            {
                break;
            }
            i = i - left;
        }

        break;
    }

    // Removing the GVD in the corners, so we just have the points in the middle of each room
    if (countLeft == countRight || countUp == countDown)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Checking if the GVD is in a valid room. An invalid room is outside the walls in the most outer walls
bool PathPlanning::isValidRoom(int index, std::vector<cellValueBF> cellVector)
{
    int up = 240;
    int right = 1;
    int down = 240;
    int left = 1;

    // Checks if the current cell's left and right neighbours are both wall -> Then it is not a valid room
    if ((index + right) % 240 != 0 && (index - left) % 240 != 239)
    {
        if (cellVector[index + right].value == 1 && cellVector[index - left].value == 1)
        {
            return false;
        }
    }

    // Checks if the current cell's up and down neighbours are both wall -> Then it is not a valid room
    if (index - up >= 0 && index + down < cellVector.size())
    {
        if (cellVector[index - up].value == 1 && cellVector[index + down].value == 1)
        {
            return false;
        }
    }

    // Checks if there is a outer wall
    int i = index;
    int countUp = 0;
    int countRight = 0;
    int countDown = 0;
    int countLeft = 0;

    while(true)
    {
        while (true)
        {
            countUp++;
            if (i - up < 0) // upper neighbor
            {
                return false;
            } 
            if (cellVector[i - up].value == 1)
            {
                break;
            }
            i = i - up;
        }

        i = index;
        while (true)
        {
            countRight++;
            if ((i + right) % 240 == 0) // right neighbor
            {
                return false;
            } 
            if (cellVector[i + right].value == 1)
            {
                break;
            }
            i = i + right;
        }

        i = index;
        while (true)
        {
            countDown++;
            if (i + down >= cellVector.size()) // down neighbor
            {
                return false;
            } 
            if (cellVector[i + down].value == 1)
            {
                break;
            }
            i = i + down;
        }

        i = index;
        while (true)
        {
            countLeft++;
            if ((i - left) % 240 == 239) // left neighbor
            {
                return false;
            } 
            if (cellVector[i - left].value == 1)
            {
                break;
            }
            i = i - left;
        }

        break;
    }

    return true;
}

PathPlanning::~PathPlanning()
{
}
