
#include "Localization.h"

Localization::Localization()
{
}

/*void Localization::particleFilter()
{
    generateParticles(1000);

    double landmarks[5][2] = { {5, 2}, {10, 5}, {40, 30}, {80, 30}, {10, 70} };

    // Start coordinates (center of the map)
    int posX = 60;
    int posY = 40;
    int yaw = 0;
}

std::vector<particle> Localization::generateParticles(int _numberOfParticles)
{
    numberOfParticles = _numberOfParticles;

    std::vector<particle> particleVector;

    // Start at 0. Standard deviation equal to 1
    std::normal_distribution<double> normal_x(0, 1);
    std::normal_distribution<double> normal_y(0, 1);
    std::normal_distribution<double> normal_yaw(0, 1);

    for (int i = 0; i < numberOfParticles; i++)
    {
        particle p;
        p.id = i;

		double x_rand = rand() % 120 + 1;
        double y_rand = rand() % 80 + 1;
        p.coord = cv::Point2f(x_rand, y_rand);

        double yaw_rand = (double) rand() / (double) RAND_MAX;
        yaw_rand *= 2*M_PI;
        p.yaw = yaw_rand;

        p.weight = 1;

        particleVector.push_back(p);
        //std::cout << "ID: " << p.id << " Coord: " << p.coord.x << "," << p.coord.y << "     Yaw: " << p.yaw << "    Weight: " << p.weight << std::endl;
    }

    return particleVector;
}*/

std::vector<double> Localization::lidarDistance(cv::Point2f pixel)
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);
    int resize = 1;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows));

    double angleMin = -2.26889;
    double angleIncrement = 0.022803;
    double scaling = 1.41735;
    double maxDistance = 10 * scaling;
    int nranges = 200;
    double rangeIncrement = 0.001;

    std::vector<double> lidarVector;

    for (int i = 0; i < nranges; i++)
    {
        double x = pixel.x;
        double y = pixel.y;
        double angle = angleMin + i * angleIncrement;
        double range = 0;
        
        if (angle < -0.5*M_PI)
        {
            angle = -(M_PI + angle);
        }
        else if (angle > 0.5*M_PI)
        {
            angle = M_PI - angle;
        }

        while (range < (maxDistance - rangeIncrement) && image.at<cv::Vec3b>(y, x)[0] > 240)
        {
            range += rangeIncrement;
            x = pixel.x;
            y = pixel.y;

            if (angle > -0.5*M_PI && angle < 0) // 4th quadrant
            {
                // Negativ sign, because of negative angle (-*- = +)
                x += cos(angle) * range;
                y += -sin(angle) * range;
            }
            else if (angle < 0.5*M_PI && angle > 0) // 1st quadrant
            {
                // y is negative, because we go upwards (negative in an image)
                x += cos(angle) * range;
                y += -sin(angle) * range;
            }
            else if (angle < -0.5*M_PI) // 3rd quadrant
            {
                x += -cos(angle) * range;
                y += -sin(angle) * range;               
            }
            else if (angle > 0.5*M_PI) // 2nd quadrant
            {
                x += -sin(angle) * range;
                y += -cos(angle) * range;              
            }
        }

        lidarVector.push_back(range);
        std::cout << "i: " << i << " Range: " << range/scaling << std::endl;
    }

    return lidarVector;
}

Localization::~Localization()
{
}