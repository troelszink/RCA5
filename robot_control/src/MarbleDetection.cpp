
#include "MarbleDetection.h"


MarbleDetection::MarbleDetection()
{

}

void MarbleDetection::cameraCallback(ConstImageStampedPtr &msg) 
{
    static boost::mutex mutex;

    std::size_t width = msg->image().width(); // 320 pixels
    std::size_t height = msg->image().height(); // 240 pixels
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    im = im.clone();
    cv::cvtColor(im, im, CV_RGB2BGR);

    cv::Mat imPre = preprocessing(im);
    cv::Mat result = houghCircles(imPre);

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
    cv::GaussianBlur(imGray, imGray, cv::Size(15, 15), 0, 0, cv::BORDER_DEFAULT);

    return imGray;
}

cv::Mat MarbleDetection::binaryThreshold(cv::Mat im)
{
    //cv::Mat imGraySplit[3];
    //cv::split(imGray, imGraySplit);
    double luminanceThreshLow = 90;
    double luminanceThreshHigh = 180;
    double luminance_maxValue = 255;
    cv::Mat luminanceBinaryLow, luminanceBinaryHigh, luminanceBinary;
    cv::threshold(im, luminanceBinaryLow, luminanceThreshLow, luminance_maxValue, CV_THRESH_BINARY);
    cv::threshold(im, luminanceBinaryHigh, luminanceThreshHigh, luminance_maxValue, CV_THRESH_BINARY_INV);
    cv::bitwise_and(luminanceBinaryLow, luminanceBinaryHigh, luminanceBinary);

    return luminanceBinary;
}

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
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    cv::Sobel( im, grad_x, ddepth, 1, 0, 1, scale, delta, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    cv::Sobel( im, grad_y, ddepth, 0, 1, 1, scale, delta, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

    //cv::Mat gradL, gradU;
    //cv::threshold(grad, grad, 30, 255, CV_THRESH_BINARY);
    //cv::threshold(grad, gradU, 80, 255, CV_THRESH_BINARY_INV);
    //cv::bitwise_and(gradL, gradU, grad);

    return grad;
}

cv::Mat MarbleDetection::houghCircles(cv::Mat im)
{

    // Detection of the marble
    //cv::Mat imDet = binaryThreshold(im);
    cv::Mat imDet = edgeDetection(im);

    static cv::Point current(-1, -1);
    int width = 320;
    int height = 240;

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    //
    cv::HoughCircles(imDet, circles, CV_HOUGH_GRADIENT,
                 1,   // accumulator resolution (size of the image / 2)
                 3000,  // minimum distance between two circles
                 60, // Canny high threshold // 350
                 10, // minimum number of votes // 10
                 1, 200); // min and max radius

    /// Draw the circles detected
    for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            cv::circle(imDet, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            cv::circle(imDet, center, radius, cv::Scalar(255, 255, 255), 3, 8, 0);
            int diameter = 2 * radius;

            if (current != center)
                {
                    //std::cout << "Position of the white ball is:" << center << " with diameter: " << diameter << std::endl;
                    current = center;

                    marbleLocation(diameter, center.x, center.y);

                    //float angle = atan2(0, center.x - width/2) * 180/M_PI;
                    //std::cout << "Angle: " << angle << std::endl;
                }
            //cv::waitKey();
        }

    return imDet;
}

void MarbleDetection::addObject(Callback &obj)
{
    callback = &obj;
}

void MarbleDetection::marbleLocation(float marbleWidth, float centerX, float centerY)
{
    // Determining the angle to the marble
    float distanceToMarbleHor = (2 * marbleRadius * focalLength) / marbleWidth;
    float opposite = (centerX - cameraWidth/2) / cameraWidth * FOV; // Also converting to real distance (instead of pixels)
    float marbleAngle = atan(opposite / distanceToMarbleHor)/* * 180/M_PI*/;
    float angle = marbleAngle + callback->getYaw();

    // The actual distance to the marble (with angle)
    float distanceToMarble = sqrt(pow(distanceToMarbleHor, 2) + pow(opposite, 2));

    // Location
    float x2 = callback->getCurPosition().x + distanceToMarble * cos(angle); // Equations found in Mathematica
    float y2 = callback->getCurPosition().y + distanceToMarble * sin(angle) - opposite;

    std::cout << "DistanceToMarble: " << distanceToMarble << std::endl;
    std::cout << "The location of the marble is: (" << x2 << "," << y2 << ")" << std::endl;
    //std::cout << callback->getCurPosition().x << "," << callback->getCurPosition().y << std::endl;

    //std::cout << "Distance: " << distanceToMarble << std::endl;
    //std::cout << "Angle : " << marbleAngle << std::endl;
}

MarbleDetection::~MarbleDetection()
{
    
}