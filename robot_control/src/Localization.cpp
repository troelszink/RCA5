
#include "Localization.h"

Localization::Localization()
{
    nCoordinates = 0;
}

cv::Point2f Localization::localize(std::vector<float> rangeVector)
{
    //  std::vector<particle> particleVector = updateWeigths(generateParticles(100), rangeVector);

    //displayParticles(particleVector);

    cv::Point2f point;

    return point;
}

std::vector<particle> Localization::generateParticles(int _numberOfParticles)
{
    numberOfParticles = _numberOfParticles;

    std::vector<particle> particleVector;

    /*// Start at 0. Standard deviation equal to 1
    std::normal_distribution<float> normal_x(0, 1);
    std::normal_distribution<float> normal_y(0, 1);
    std::normal_distribution<float> normal_yaw(0, 1);*/

    // We set a particle's position to be equal to the robots initial position.
    /*particle p;
    p.id = 0;
    float x = 0;
    float y = 0;
    p.coord = cv::Point2f(x, y);
    p.yaw = 0;
    p.weight = 1;
    particleVector.push_back(p);*/

    particle p;
    // Generates the rest of the particles
    for (int i = 0; i < numberOfParticles; i++)
    {
        p.id = i;

		//float x_rand = rand() % 120 + 1;
        //float y_rand = rand() % 80 + 1;
        // The global center of our environment
        float x_rand = 60;
        float y_rand = 40;
        p.coord = cv::Point2f(x_rand, y_rand);

        float yaw_rand = (float) rand() / (float) RAND_MAX;
        yaw_rand *= 2*M_PI - M_PI;
        p.yaw = yaw_rand;

        p.weight = 1;

        //p.lidarData = lidarDistance(p.coord, p.yaw);

        particleVector.push_back(p);
        //std::cout << "ID: " << p.id << " Coord: " << p.coord.x << "," << p.coord.y << "     Yaw: " << p.yaw << "    Weight: " << p.weight << std::endl;
    }

    return particleVector;
}

std::vector<float> Localization::lidarDistance(cv::Point2f pixel, float yaw)
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);
    int resize = 1;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows));

    float angleMin = -2.26889 + yaw;
    float angleIncrement = 0.022803;
    float scaling = 1.41735;
    float maxDistance = 10 * scaling;
    int nranges = 200;
    float rangeIncrement = 0.001;

    std::vector<float> lidarVector;

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
        //std::cout << "i: " << i << " Range: " << range/scaling << std::endl;
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
        //std::cout << "Size of rangeVector: " << rangeVector.size() << std::endl;

        float sigma = 1;
        float sumOfWeigths = 0;

        // Add noise according to a uniform distribution
        //std::default_random_engine generator;
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_real_distribution<double> distribution(-0.5, 0.5);

        for (int i = 0; i < particleVector.size(); i++)
        {
            particleVector[i].lidarData = lidarDistance(particleVector[i].coord, particleVector[i].yaw);
            //particleVector[i].coord.x += distribution(generator);
            //particleVector[i].coord.y += distribution(generator);
            particleVector[i].yaw += distribution(generator); //(float) rand() / (float) RAND_MAX * 0.5 - 0.25;
            //std::cout << (float) rand() / (float) RAND_MAX * 0.5 -0.25 << std::endl;

            // Generating noise in proportion to which quadrant the particle are facing outwards to
            if (particleVector[i].yaw >= 0 && particleVector[i].yaw < 0.5*M_PI) // 1st quadrant
            {
                particleVector[i].coord.x += cos(particleVector[i].yaw) * distribution(generator); //(float) rand() / (float) RAND_MAX * 0.5 - 0.25;
                particleVector[i].coord.y += -sin(particleVector[i].yaw) * distribution(generator); //(float) rand() / (float) RAND_MAX * 0.5 - 0.25;
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

            for (int j = 0; j < rangeVector.size(); j++)
            {
                float d = rangeVector[j] * scaling;
                float y = particleVector[i].lidarData[j];

                // General Normal Distribution
                sum += 1/(sigma*sqrt(2*M_PI)) * exp(-pow(y-d, 2) / (2*pow(sigma, 2)));

                //std::cout << "LidarData: " << d << std::endl;
            }
            
            sum /= rangeVector.size();
            particleVector[i].weight = sum;
            sumOfWeigths += sum;

        }

        // Normalizing
        for (int i = 0; i < particleVector.size(); i++)
        {
            particleVector[i].weight /= sumOfWeigths;

            //std::cout << "Coord: " << particleVector[i].coord.x << "," << particleVector[i].coord.y << " Weight: " << particleVector[i].weight << std::endl;
        }

        // Resampling
        particleVector = resample(particleVector);

        displayParticles(particleVector);

        saveCoords(particleVector, curPosition);
    }

    //std::cout << "P(x): " << p << std::endl;

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
            //i = i % N + 1;
            i++;
            if (i >= particleVector.size())
            {
                //std::cout << "I too high" << std::endl;  
                i = 0;
            }
            c += particleVector[i].weight;
        }

        newParticleVector.push_back(particleVector[i]);
        //std::cout << "i: " << i << std::endl;
    }
    //std::cout << "Size: " << newParticleVector.size() << std::endl;

    return newParticleVector;
}

void Localization::displayParticles(std::vector<particle> particleVector)
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);
    int resize = 5;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, cv::INTER_NEAREST);

    for (int i = 0; i < particleVector.size(); i++)
    {
        cv::Point2f coord = particleVector[i].coord;

        image.at<cv::Vec3b>(coord.y * resize, coord.x * resize)[0] = 0;
        image.at<cv::Vec3b>(coord.y * resize, coord.x * resize)[1] = 0;
        image.at<cv::Vec3b>(coord.y * resize, coord.x * resize)[2] = 255;
    }

    resize = 2;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, cv::INTER_NEAREST);
    //std::cout << particleVector[0].coord.x << "," << particleVector[0].coord.y << std::endl;

    cv::imwrite("../testImages/BigWorldV2-Localization.png", image );
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

    float robotX = curPosition.x * scaling + 60; // 60 is the center of the image in pixels (x-direction)
    float robotY = -(curPosition.y * scaling) + 40; // 40 is the center of the image in pixels (y-direction)

    std::vector<float> coordinates;

    coordinates.push_back(robotX);
    coordinates.push_back(robotY);
    coordinates.push_back(averageX);
    coordinates.push_back(averageY);

    // Push these coordinates onto a vector
    coordinatesVector.push_back(coordinates);

    if (nCoordinates == 55) // Choose number of coordinates to generate
    {
        createCSVfile(coordinatesVector);
    }
 }

void Localization::createCSVfile(std::vector<std::vector<float>> _coordinatesVector)
 {
    // file pointer 
    std::fstream fout; 
  
    // opens an existing csv file or creates a new file. 
    fout.open("../otherFiles/Coordinates2.csv", std::ios::out | std::ios::app); 
  
    // Insert the data to the file 
    fout << _coordinatesVector[0][0] << "; "
         << _coordinatesVector[0][1] << "; "
         << 60 << "; "
         << 40
         << "\n"; 

    for (int i = 1; i < _coordinatesVector.size(); i++)
    {
        // Insert the data to the file 
        fout << _coordinatesVector[i][0] << "; "
             << _coordinatesVector[i][1] << "; "
             << _coordinatesVector[i - 1][2] << "; "
             << _coordinatesVector[i - 1][3]
             << "\n"; 
    }

    std::cout << "Done creating CSV-file!" << std::endl;
 }

Localization::~Localization()
{
}