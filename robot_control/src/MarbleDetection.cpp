
#include "MarbleDetection.h"


MarbleDetection::MarbleDetection()
{

}

// The built-in method supplied with our own code
void MarbleDetection::cameraCallback(ConstImageStampedPtr &msg) 
{
    static boost::mutex mutex;

    std::size_t width = msg->image().width(); // 320 pixels
    std::size_t height = msg->image().height(); // 240 pixels
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    im = im.clone();
    cv::cvtColor(im, im, CV_RGB2BGR);

    // Calling methods to implement our design
    cv::Mat imPre = preprocessing(im);
    cv::Mat imDet = edgeDetection(imPre);
    cv::Mat result = houghCircles(imDet;

    mutex.lock();
    cv::imshow("camera", result);
    mutex.unlock();
}

cv::Mat MarbleDetection::preprocessing(cv::Mat im)
{
    cv::Mat imGray;
    imGray = im.clone();
    cv::cvtColor(im, imGray, CV_BGR2GRAY);

    // Reduce the noise
    cv::Mat imDenoising;
    cv::fastNlMeansDenoising(imGray, imDenoising, 10, 7, 11);

    return imDenoising;
}

// Generate edge detection by means of gradients and Sobel derivatives
cv::Mat MarbleDetection::edgeDetection(cv::Mat im)
{
    cv::Mat grad;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    /// Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    cv::Sobel( im, grad_x, ddepth, 1, 0, 1, scale, delta, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    cv::Sobel( im, grad_y, ddepth, 0, 1, 1, scale, delta, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

    return grad;
}

// Draw the Hough cirlces around the marbles
cv::Mat MarbleDetection::houghCircles(cv::Mat imDet)
{
    // Calculing the Canny value used in HoughCircles()
    cv::Mat imCanny;
    double canny_value = cv::threshold(imDet, imCanny, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    static cv::Point current(-1, -1);
    int width = 320;
    int height = 240;
    int iterations = 10;

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    //
    cv::HoughCircles(imDet, circles, CV_HOUGH_GRADIENT,
                 1,   // accumulator resolution (size of the image / 2)
                 3000,  // minimum distance between two circles
                 canny_value, // Canny high threshold
                 30, // minimum number of votes
                 1, 200); // min and max radius

    /// Draw the circles detected
    for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            cv::circle(imDet, center, 3, cv::Scalar(255, 255, 255), -1, 8, 0);
            // circle outline
            cv::circle(imDet, center, radius, cv::Scalar(255, 255, 255), 3, 8, 0);
            int diameter = 2 * radius;

            countDiameter++;
            sumDiameter += diameter;
            sumX += center.x;
            sumY += center.y;
            
            if (countDiameter == iterations)
            {
                diameter = sumDiameter / iterations;
                center.x = sumX / iterations;
                center.y = sumY / iterations;

                if (current != center)
                    {
                        current = center;
                        // Calling the method that estimates the location of the marbles
                        marbleLocation(diameter, center.x, center.y);
                    }

                countDiameter = 0;
                sumDiameter = 0;
                sumX = 0;
                sumY = 0;
            }
        }

    return imDet;
}

// This method is used to get the information from the same object used in the Callback class.
// So we can get the correct data from the lidar-sensors to compare with
void MarbleDetection::addObject(Callback &obj)
{
    callback = &obj;
}

void MarbleDetection::marbleLocation(float marbleWidth, float centerX, float centerY)
{
    // Determining the angle to the marble
    float distanceToMarbleHor = (2 * marbleRadius * focalLength) / marbleWidth;
    float opposite = (centerX - cameraWidth/2) / cameraWidth * FOV; // Also converting to real distance (instead of pixels) - FOV is the angle in radians   
    float marbleAngle = atan(opposite / distanceToMarbleHor)/* * 180/M_PI*/;
    float angle = marbleAngle + callback->getYaw();

    // The actual distance to the marble (with angle)
    float distanceToMarble = sqrt(pow(distanceToMarbleHor, 2) + pow(opposite, 2));

    float x2, y2;

    // Location
    // Between 4th and 1st quadrant
    if (callback->getYaw() < 0.25*M_PI && callback->getYaw() >= -0.25*M_PI)
    {
        x2 = callback->getCurPosition().x + distanceToMarble * cos(angle); // Equations found in Mathematica
        y2 = callback->getCurPosition().y - distanceToMarble * sin(angle);
    }
    // Between 1st and 2nd quadrant
    else if(callback->getYaw() < 0.75*M_PI && callback->getYaw() >= 0.25*M_PI)
    {
        x2 = callback->getCurPosition().x + distanceToMarble * sin(angle); // Equations found in Mathematica
        y2 = callback->getCurPosition().y + distanceToMarble * cos(angle);
    }
    // Between 2nd and 3rd quadrant
    else if(callback->getYaw() < -0.75*M_PI && callback->getYaw() >= 0.75*M_PI)
    {
        x2 = callback->getCurPosition().x - distanceToMarble * cos(angle); // Equations found in Mathematica
        y2 = callback->getCurPosition().y + distanceToMarble * sin(angle);
        //std::cout << "2nd and 3rd" << std::endl;
    }
    // Between 3rd and 4th quadrant
    else if (callback->getYaw() < -0.25*M_PI && callback->getYaw() >= -0.75*M_PI)
    {
        x2 = callback->getCurPosition().x - distanceToMarble * sin(angle); // Equations found in Mathematica
        y2 = callback->getCurPosition().y - distanceToMarble * cos(angle);
    }

    //std::cout << "DistanceToMarble: " << distanceToMarble << std::endl;
    std::cout << "The location of the marble is: (" << x2 << "," << y2 << ")" << std::endl;
    std::cout << "The location of the robot is: (" << callback->getCurPosition().x << "," << callback->getCurPosition().y << ")" << std::endl;
}

// Just for drawing our tests
void MarbleDetection::drawTest()
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);
    float scaling = 1.41735;
    cv::Point2f center = cv::Point2f(60, 40);

    // Draw the scenarios - Positions found in Gazebo and with the camera

    // Scenario 1
    float S1_x = 21.5 * scaling + center.x;
    float S1M_y = -(0.25 * scaling) + center.y;
    float S1R_y = -(2.22 * scaling) + center.y;
    image.at<cv::Vec3b>(S1M_y, S1_x)[0] = 0;
    image.at<cv::Vec3b>(S1M_y, S1_x)[1] = 0;
    image.at<cv::Vec3b>(S1M_y, S1_x)[2] = 255; // Red for marble
    image.at<cv::Vec3b>(S1R_y, S1_x)[0] = 255; // Blue for robot
    image.at<cv::Vec3b>(S1R_y, S1_x)[1] = 0;
    image.at<cv::Vec3b>(S1R_y, S1_x)[2] = 0;
    // Scenario 2
    float S2_x = 21.5 * scaling + center.x;
    float S2R_y = -(4.51 * scaling) + center.y;
    image.at<cv::Vec3b>(S2R_y, S2_x)[0] = 255;
    image.at<cv::Vec3b>(S2R_y, S2_x)[1] = 0;
    image.at<cv::Vec3b>(S2R_y, S2_x)[2] = 0;
    // Scenario 3
    float S3_x = 21.5 * scaling + center.x;
    float S3R_y = -(11.13 * scaling) + center.y;
    image.at<cv::Vec3b>(S3R_y, S3_x)[0] = 255;
    image.at<cv::Vec3b>(S3R_y, S3_x)[1] = 0;
    image.at<cv::Vec3b>(S3R_y, S3_x)[2] = 0;
    // Scenario 4
    float S4R_x = -(12.57 * scaling) + center.x;
    float S4R_y = -(11.0 * scaling) + center.y;
    float S4M1_x = -12.57 * scaling + center.x;
    float S4M1_y = -(18.5 * scaling) + center.y;
    float S4M2_x = -(9.07 * scaling) + center.x;
    float S4M2_y = -(18.5 * scaling) + center.y;
    float S4M3_x = -(9.07 * scaling) + center.x;
    float S4M3_y = -(15.0 * scaling) + center.y;
    image.at<cv::Vec3b>(S4M1_y, S4M1_x)[0] = 0;
    image.at<cv::Vec3b>(S4M1_y, S4M1_x)[1] = 0;
    image.at<cv::Vec3b>(S4M1_y, S4M1_x)[2] = 255;
    image.at<cv::Vec3b>(S4M2_y, S4M2_x)[0] = 0;
    image.at<cv::Vec3b>(S4M2_y, S4M2_x)[1] = 0;
    image.at<cv::Vec3b>(S4M2_y, S4M2_x)[2] = 255;
    image.at<cv::Vec3b>(S4M3_y, S4M3_x)[0] = 0;
    image.at<cv::Vec3b>(S4M3_y, S4M3_x)[1] = 0;
    image.at<cv::Vec3b>(S4M3_y, S4M3_x)[2] = 255;
    image.at<cv::Vec3b>(S4R_y, S4R_x)[0] = 255;
    image.at<cv::Vec3b>(S4R_y, S4R_x)[1] = 0;
    image.at<cv::Vec3b>(S4R_y, S4R_x)[2] = 0;
    // Scenario 5
    float S5_x = 4.44 * scaling + center.x;
    float S5M_y = -(4.35 * scaling) + center.y;
    float S5R_y = -(2.43 * scaling) + center.y;
    image.at<cv::Vec3b>(S5M_y, S5_x)[0] = 0;
    image.at<cv::Vec3b>(S5M_y, S5_x)[1] = 0;
    image.at<cv::Vec3b>(S5M_y, S5_x)[2] = 255;
    image.at<cv::Vec3b>(S5R_y, S5_x)[0] = 255;
    image.at<cv::Vec3b>(S5R_y, S5_x)[1] = 0;
    image.at<cv::Vec3b>(S5R_y, S5_x)[2] = 0;

    // Resize the image
    float resize = 10;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, cv::INTER_NEAREST);

    cv::imwrite( "../testImages/BigWorldV2-MarbleDetection.png", image );

    cv::namedWindow("Marble Detection", CV_WINDOW_AUTOSIZE);
    cv::imshow("Marble Detection", image);

    cv::waitKey();
}

MarbleDetection::~MarbleDetection()
{
    
}
