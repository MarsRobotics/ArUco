#include <iostream>
#include <aruco/aruco.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <time.h>

#define EVER (;1;)

using namespace cv;
using namespace aruco;


//Width of line to highlight marker with
const int LINE_WIDTH = 1;

//Color to highlight marker lines with
const Scalar COLOR = (0, 0, 255);

//Delay before next image retrieval
const int WAIT_TIME = 2;

//Integer representation of which keyboard key was pressed
int inputKey = 0;

//Path to files
string boardConfigFile;

//Video capture components
VideoCapture videoCapture;
Mat inputImage,inputImageCopy;

//Board detection components
BoardDetector boardDetector;
BoardConfiguration boardConfig;

//Setup the camera parameters
double camData[] = {1.3089596458327883e+03, 0.0, 3.1716402612677911e+02, 0.0, 1.3249652390726415e+03, 2.3359932814285278e+02, 0., 0., 1.0};
Mat cam = Mat(3, 3, CV_32F, camData);
double distortionData[] = {1.8869609562810794e+00, 4.2431486029577115e+01,
        -7.9002380674102313e-02, -1.0615309141897006e-02,
        -1.2940788383601671e+03};
Mat distortion = Mat(1, 5, CV_32F, distortionData);
Size size = Size(640, 480);
CameraParameters cp = CameraParameters(cam, distortion, size);

//Set of markers detected
Vector<Marker> markers;

//Maximum distance between any two corners for the most recent marker scanned
double cornerDistance = 0;

//Sum of marker calibration components Sum(corner Distance * Actual Distance)
double markerSum = 0;

//Number of values included in markerSum
int markerCount = 0;

//Weather or not to close the program.
char closeTheProgram = 0;

/**
* consoleInput
*
* Description: Thread function to read measured distance values from stdin and preform the calibration calculations
*
*/
void* consoleInput(void*)
{
    //Get a timestamp to name the log file
    time_t now;
    time(&now);
    char formattedTime [80];
    strftime(formattedTime, 80, "%F %T", localtime(&now));

    //avoid the hastle of dealing with char*
    string logFileName ("");
    logFileName += "distance_calibration ";
    logFileName += formattedTime;
    logFileName += ".log";

    //Open a log file and write a header
    ofstream logFile (logFileName.c_str());
    logFile << "\"corner distance\" \"measured distance\"" << endl;

    //Loop until the user exits the program
    for EVER {

        //Get a measured value from the user
        cout << "Enter the distance from camera to marker (-1 to exit): ";
        double measuredDistance;
        cin >> measuredDistance;
        cout << endl;

        //Check if the value is the exit value
        if (measuredDistance == -1) {
            cout << "User terminated program. Exiting." << endl;
            break;
        }

        //Check if the measured corner distance is invalid
        if (cornerDistance < 0) {
            cout << "No markers are detected. Data not recorded." << endl;
            continue;
        }

        //log the data
        logFile << cornerDistance << " " << measuredDistance << endl;

        //Update and display the constant produced by the calibration
        markerSum += cornerDistance * measuredDistance;
        ++markerCount;
        cout << "updated constant: " << (markerSum / markerCount) << endl;

    }

    //close the log file
    logFile.close();

    //tell the other threads to close
    closeTheProgram = 1;

    return NULL;
}


int main(int argc,char **argv)
{

    //Verify that the correct inputs have been provided
    if (argc!=3) {
        cerr << "Usage: boardConfig.yml camera_config.yml " << endl;
        return -1;
    }

    //Launch the thread to read console input
    pthread_t consoleInputThread;
    pthread_create(&consoleInputThread, NULL, &consoleInput, NULL);

    try
    {

        //Read the board
        boardConfigFile = argv[1];
        boardConfig.readFromFile(boardConfigFile);

        //Open the camera
        videoCapture.open(0);

        //make sure camera is open
        if (!videoCapture.isOpened()) {
            cerr<<"Could not open the video"<<endl;
            return -1;
        }

        //read an image from camera
        videoCapture >> inputImage;

        //create the gui
        cv::namedWindow("output video", CV_WINDOW_AUTOSIZE);

        //Configure the board detector
        boardDetector.setParams(boardConfig);


        //While there is a frame available (video hasn't ended/camera is still plugged in)
        while ( videoCapture.grab()) {

            //If the other thread has closed then close
            if (closeTheProgram)
                break;

            //retrieve an image from the camera
            videoCapture.retrieve(inputImage);
            inputImage.copyTo(inputImageCopy);

            //detect markers in the frame
            boardDetector.detect(inputImage);
            markers = boardDetector.getDetectedMarkers();

            //If no markers are detected then invalidate the data shared with the other thread so it will ignore it
            if (markers.size() == 0) {
                cornerDistance = -1;
            }

            //Loop through all the markers
            for (unsigned int mi = 0; mi < markers.size(); mi++) {
                Marker m = markers[mi];

                //Find the longest distance between two any corners of the marker
                double maxCornerDist = 0;
                for (int i=0;i<4;i++) {
                    for (int j=i+1;j<4;j++) {
                        double dist = sqrt(pow(m[i].x-m[j].x, 2) + pow(m[i].y-m[j].y, 2));
                        if (dist > maxCornerDist) {
                            maxCornerDist = dist;
                        }
                    }
                }

                //Make the maximum corner distance available to the other thread
                cornerDistance = maxCornerDist;

                //Display the marker on the image
                m.draw(inputImageCopy, COLOR, LINE_WIDTH);

            }

            //Display the image with the gui
            cv::imshow("output video",inputImageCopy);


            //Check if the stop button has been pressed
            inputKey = cv::waitKey(WAIT_TIME);
        }

        return 0;

    } catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
    return 0;
}


