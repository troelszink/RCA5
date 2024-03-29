
#include "Localization.h"

Localization::Localization()
{
    nCoordinates = 0;
}

std::vector<particle> Localization::generateParticles(int _numberOfParticles)
{
    numberOfParticles = _numberOfParticles;

    std::vector<particle> particleVector;

    particle p;
    // Generate the particles
    for (int i = 0; i < numberOfParticles; i++)
    {
        p.id = i;
	
	// All particles have the same start position as the initial position of the robot
        float x_rand = 60;
        float y_rand = 40;
        p.coord = cv::Point2f(x_rand, y_rand);

	// Generate the yaw of the particles according to a normal distribution
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_real_distribution<double> distribution(-M_PI, M_PI);
        p.yaw = distribution(generator);

	// All particles is equal likely to be correct when generated, therefore weight is equal to 1
        p.weight = 1;

        particleVector.push_back(p);
    }

    return particleVector;
}

// Calculates the lidar-data from every particle. This is used to compare with the data given from Gazebo
std::vector<float> Localization::lidarDistance(cv::Point2f pixel, float yaw)
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);
    int resize = 1;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows));

    // Defining usefull variables
    float angleMin = -2.26889 + yaw;
    float angleIncrement = 0.022803;
    float scaling = 1.41735;
    float maxDistance = 10 * scaling;
    int nranges = 200;
    float rangeIncrement = 0.001;

    std::vector<float> lidarVector;

    // Checking each lidar-sensor (every 10th, decreased the amount of sensors from 200 to 21)
    for (int i = 0; i < nranges; i+=10) // Changed from i++ to i+=10
    {
        float x = pixel.x;
        float y = pixel.y;
        float angle = angleMin + i * angleIncrement;
        float range = 0;
        
        if (angle < -0.5*M_PI)
        {
            angle = -(M_PI + angle);
        }
        else if (angle > 0.5*M_PI)
        {
            angle = M_PI - angle;
        }

	// As long as the range is smaller than max distance (minus one range increment) and the pixel is white, do the following
        while (range < (maxDistance - rangeIncrement) && image.at<cv::Vec3b>(y, x)[0] > 240)
        {
            range += rangeIncrement;
            x = pixel.x;
            y = pixel.y;

	    // Checking special cases for the four different quadrants
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
    }

    return lidarVector;
}

// See Algorithm 17 in the book
std::vector<particle> Localization::updateWeigths(std::vector<particle> particleVector, std::vector<float> rangeVector, cv::Point2f curPosition)
{
    int measurements = 1;
    float scaling = 1.41735;

    for (int m = 0; m < measurements; m++)
    {   
        std::cout << "Update Weights" << std::endl;

        float sigma = 0.5; // For the General Normal Distribution
        float sumOfWeigths = 0;

        // Add noise according to a normal distribution
        std::random_device rd;
        std::mt19937 generator(rd());
        int mean = 0;
        float stdDev = 0.1;
        std::normal_distribution<double> distribution(mean, stdDev);

	// Changing the position, yaw and lidar-data for each particle
        for (int i = 0; i < particleVector.size(); i++)
        {
            particleVector[i].lidarData = lidarDistance(particleVector[i].coord, particleVector[i].yaw);
            particleVector[i].yaw += distribution(generator);

            // Generating noise in proportion to which quadrant the particle are facing outwards to
            if (particleVector[i].yaw >= 0 && particleVector[i].yaw < 0.5*M_PI) // 1st quadrant
            {
                particleVector[i].coord.x += cos(particleVector[i].yaw) * distribution(generator);
                particleVector[i].coord.y += -sin(particleVector[i].yaw) * distribution(generator);
            } 
            else if (particleVector[i].yaw >= 0.5*M_PI && particleVector[i].yaw < M_PI) // 2nd quadrant
            {
                particleVector[i].coord.x += -cos(M_PI - particleVector[i].yaw) * distribution(generator);
                particleVector[i].coord.y += -sin(M_PI - particleVector[i].yaw) * distribution(generator);
            }
            else if (particleVector[i].yaw >= -M_PI && particleVector[i].yaw < -0.5*M_PI) // 3rd quadrant
            {
                particleVector[i].coord.x += -cos(M_PI + particleVector[i].yaw) * distribution(generator);
                particleVector[i].coord.y += sin(M_PI + particleVector[i].yaw) * distribution(generator);
            }
            else if (particleVector[i].yaw >= -0.5*M_PI && particleVector[i].yaw < 0) // 4th quadrant
            {
                particleVector[i].coord.x += cos(particleVector[i].yaw) * distribution(generator);
                particleVector[i].coord.y += -sin(particleVector[i].yaw) * distribution(generator); // Negative "-" as we need y to be positive, but it becomes negative, because of negative yaw
            }

            float sum = 0;

	    // Calculating the weight of each particle according to the general normal distribution
            for (int j = 0; j < rangeVector.size(); j++)
            {
                float y = rangeVector[j] * scaling;
                float d = particleVector[i].lidarData[j];

                // General Normal Distribution
                sum += 1/(sigma*sqrt(2*M_PI)) * exp(-pow(y-d, 2) / (2*pow(sigma, 2)));
            }
            
            sum /= rangeVector.size();
            particleVector[i].weight = sum;
            sumOfWeigths += sum;
        }

        // Normalizing
        for (int i = 0; i < particleVector.size(); i++)
        {
            particleVector[i].weight /= sumOfWeigths;
        }

        // Resampling
        particleVector = resample(particleVector);

	// Show the particles in the environment
        displayParticles(particleVector);

	// Save the coordinates of location of the actual robot and the estimated location - Used to make a graph in MATLAB
        saveCoords(particleVector, curPosition);
    }

    return particleVector;
}

// See Algorithm 18 in the book
std::vector<particle> Localization::resample(std::vector<particle> particleVector)
{
    std::cout<<"Resample" << std::endl;

    std::vector<particle> newParticleVector;

    float N = particleVector.size();
    float delta = rand() % 1/N + 1/N;

    for (int j = 0; j < N; j++)
    {
        float c = particleVector[0].weight;
        int i = 0;
        float u = delta + j * 1/N;

        while (u > c)
        {
            i++;
            if (i >= particleVector.size())
            {
                i = 0;
            }
            c += particleVector[i].weight;
        }

        newParticleVector.push_back(particleVector[i]);
    }

    return newParticleVector;
}

void Localization::displayParticles(std::vector<particle> particleVector)
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);
    int resize = 5;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, cv::INTER_NEAREST);

    // Colouring the position of each particles red
    for (int i = 0; i < particleVector.size(); i++)
    {
        cv::Point2f coord = particleVector[i].coord;

        image.at<cv::Vec3b>(coord.y * resize, coord.x * resize)[0] = 0;
        image.at<cv::Vec3b>(coord.y * resize, coord.x * resize)[1] = 0;
        image.at<cv::Vec3b>(coord.y * resize, coord.x * resize)[2] = 255;
    }

    resize = 2;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, cv::INTER_NEAREST);

    cv::imwrite("../testImages/BigWorldV2-Localization2.png", image );
    cv::namedWindow("Particles", CV_WINDOW_AUTOSIZE);
	cv::imshow("Particles", image);
    cv::waitKey(10);
}

void Localization::saveCoords(std::vector<particle> particleVector, cv::Point2f curPosition)
{
    nCoordinates++;

    // Average coordinates of the particles
    float sumX, sumY = 0;
    float averageX, averageY = 0;
    int N = particleVector.size();
    float scaling = 1.41735;

    for (int i = 0; i < N; i++)
    {
        sumX += particleVector[i].coord.x;
        sumY += particleVector[i].coord.y;
    }
    averageX = sumX / N;
    averageY = sumY / N;
    
    // Using these coordinates in drawPathBWParticles()
    std::vector<float> coordinatesParticle;
    coordinatesParticle.push_back(averageX);
    coordinatesParticle.push_back(averageY);
    coordinatesParticles.push_back(coordinatesParticle);

    float robotX = curPosition.x * scaling + 60; // 60 is the center of the image in pixels (x-direction)
    float robotY = -(curPosition.y * scaling) + 40; // 40 is the center of the image in pixels (y-direction)

    std::vector<float> coordinates;

    coordinates.push_back(robotX);
    coordinates.push_back(robotY);
    coordinates.push_back(averageX);
    coordinates.push_back(averageY);

    // Push these coordinates onto a vector
    coordinatesVector.push_back(coordinates);

    if (nCoordinates == 500) // Choose number of coordinates to generate
    {
	// Create a CSV-file
        createCSVfile(coordinatesVector);
    }
}

void Localization::createCSVfile(std::vector<std::vector<float>> _coordinatesVector)
{
    // File pointer 
    std::fstream fout; 
  
    // Opens an existing csv file or creates a new file.
    // Remember to delete old file, if you are using the same name. Else there will be data from several tests in the same file
    fout.open("../otherFiles/CoordinatesNew3.csv", std::ios::out | std::ios::app); 
  
    // Insert the data to the file (the first row)
    fout << _coordinatesVector[0][0] << ", "
         << _coordinatesVector[0][1] << ", "
         << 60 << ", "
         << 40
         << "\n"; 

    // Insert the data to the file (the rest of the rows)
    for (int i = 1; i < _coordinatesVector.size(); i++)
    {
        fout << _coordinatesVector[i][0] << ", "
             << _coordinatesVector[i][1] << ", "
             << _coordinatesVector[i - 1][2] << ", "
             << _coordinatesVector[i - 1][3]
             << "\n"; 
    }

    std::cout << "Done creating CSV-file!" << std::endl;
}

void Localization::drawPathBWParticles(std::vector<std::vector<float>> position)
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);

    // Only first time
    float resize = 10;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, cv::INTER_NEAREST);

    cv::Point2f center = cv::Point2f(resize * 60, resize * 40);
    float scaling = resize * 1.41735;

    // Draw the actual path for the robot
    for (int i = 0; i < position.size(); i++)
    {
        float x = position[i][0] * scaling + center.x;
        float y = -position[i][1] * scaling + center.y;

        cv::circle(image, cv::Point(x,y), 2, cv::Scalar(0, 0, 255), 0, 1, 0);
    }

    // Draw the estimed path for the robot
    for (int i = 0; i < coordinatesParticles.size(); i++)
    {
        float x = coordinatesParticles[i][0] * resize;
        float y = coordinatesParticles[i][1] * resize;

        cv::circle(image, cv::Point(x,y), 2, cv::Scalar(255, 0, 0), 0, 1, 0);
    }

    cv::imwrite("../testImages/BigWorldV2-LocalizationAvsPNew3.png", image);

    cv::namedWindow("Path", CV_WINDOW_AUTOSIZE);
    cv::imshow("Path", image);

    cv::waitKey();
}

Localization::~Localization()
{
}
