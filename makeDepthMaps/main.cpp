#include <iostream>
#include <vector>
#include <fstream>

#include "eigen3/Eigen/Dense"
// #include "opencv4/opencv2/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <iomanip> 
#include <execution>

#include "opencv2/highgui.hpp"

using namespace std;
using namespace Eigen;
using namespace cv;

// typedef double float;
// typedef Matrix3d Matrix3f;
// typedef Vector3d Vector3f;
// typedef MatrixXd MatrixXf;
// typedef Quaterniond Quaternionf;
// #define stof stod

// typedef float float;
// typedef Matrix3f Matrix3f;
// typedef Vector3f Vector3f;
// typedef MatrixXf MatrixXf;
// typedef Quaternionf Quaternionf;

static constexpr int skipPixelsExponent = 2;
static constexpr int skipPixels = (int) pow(2, skipPixelsExponent);
static constexpr int resx = 752 / skipPixels;
static constexpr int resy = 480 / skipPixels;
static constexpr float rayEps = 0.01;
static constexpr float rayErrorLimit = 1.0 - rayEps*rayEps;

static const int counterStart = 67;
static const int counterEnd = 500;

struct Pose {
    Matrix3f R;
    Vector3f x;
    long long t;

    bool operator<(const Pose& other) {
        return t < other.t;
    }
    bool operator<(const long long& tstamp) {
        return t < tstamp;
    }
};

Matrix<float, 3, Dynamic> readPointCloud(const string& fname);
int countLines(const string& fname);
float getDepthAtRay(const Matrix<float, 3, Dynamic>& cloud, const vector<float>& depths, const Vector3f& ray);
vector<Vector3f> makeCameraRays();
vector<float> makeDepthArray(const vector<Vector3f>& camRays, const vector<Vector3f>& goodCloudVec);
vector<Pose> readCameraPoses(const string& fname);
vector<Vector3f> filterCloud(const Matrix<float,3,Dynamic>& cloud, const vector<Vector3f>& camRays);
vector<long long> readStamps(const string& fname);

int main(int argc, char const *argv[])
{
    // cout << "Skipping this many pixels: " << skipPixels << endl;

    string cloudFname = "../mav0/pointcloud0/data.ply";
    cout << "Reading pointcloud" << endl;
    Matrix<float, 3, Dynamic> pcl = readPointCloud(cloudFname);

    cout << "Making camera rays" << endl;
    vector<Vector3f> camRays = makeCameraRays();

    cout << "Reading the camera poses" << endl;
    string camFname = "../cam0Poses.csv";
    vector<Pose> camPoses = readCameraPoses(camFname);
    sort(camPoses.begin(), camPoses.end());

    // Get the closes pose to the given time
    // vector<float> stamps = {1.40371529511214E+018};
    string stampsFname = "../mav0/cam0/data.csv";
    vector<long long> stamps = readStamps(stampsFname);

    int counter = 0;

    for (const long long& stamp : stamps) {
        if (counter < counterStart) {
            ++counter;
            continue;
        } else if (counter >= counterEnd) {
            break;
        }
        
        const Pose& camPose = *lower_bound(camPoses.begin(), camPoses.end(), stamp);

        // cout << "Camera Position" << endl;
        // cout << camPose.x << endl;
        
        // cout << "Camera time" << endl;
        // cout << setprecision(50) << camPose.t << endl;

        // cout << "Stamp" << endl;
        // cout << stamp << endl;
        

        cout << "Processing Image " << counter << endl;

        // Transform the cloud to this pose
        Matrix<float, 3, Dynamic> bcl = camPose.R.transpose() * (pcl.colwise() - camPose.x );
        // bcl.colwise().normalize();

        // cout << "cloud shape: " << bcl.rows() << ", " << bcl.cols() << endl;
        // cout << "pcloud @ 10: \n" << pcl.block<3,10>(0,0) << endl;
        // cout << "bcloud @ 10: \n" << bcl.block<3,10>(0,0) << endl;

        // Making a depth array
        // cout << "Filtering the cloud" << endl;
        vector<Vector3f> goodCloudVec = filterCloud(bcl, camRays);
        // cout << "Number of visible cloud points: " << goodCloudVec.size() << endl;
        // cout << "Getting depths at rays" << endl;
        vector<float> depthArray = makeDepthArray(camRays, goodCloudVec);


        // cout << "Length of depth array: " << depthArray.size() << endl;
        // cout << "Depth array entries: \n";
        for (const auto& d : depthArray) // cout << d << ", ";
        // cout << endl;

        if(depthArray.size() == resx*resy) // check that the rows and cols match the size of your vector
        {
            Mat m = Mat(resy, resx, CV_32FC1); // initialize matrix of uchar of 1-channel where you will store vec data
            // copy vector to mat
            // memcpy(m.data, depthArray.data(), depthArray.size()*sizeof(float)); // change uchar to any type of data values that you want to use instead
            for (int r=0;r<resy;++r) {
            for (int c=0;c<resx;++c) {
                m.at<float>(r,c) = depthArray[r+c*resy];
            }   
            }
            normalize(m, m, 0, 255, NORM_MINMAX);
            m.convertTo(m, CV_8UC1);
            
            resize(m,m, Size(0,0), skipPixels, skipPixels);
            // imshow("test", m);
            // waitKey(0);
            stringstream output_fname;
            output_fname << "depths/depth_" << counter << ".png";
            imwrite(output_fname.str(), m);
        }

        // string = "image"+to_string(counter)+".pfm";
        ++counter;
    }




    // cout << "Hello depth" << endl;
    return 0;
}



Matrix<float, 3, Dynamic> readPointCloud(const string& fname) {
    // Count the lines in the file
    const int lineCount = countLines(fname);
    const int entryCount = lineCount - 11;

    Matrix<float, 3, Dynamic> result = MatrixXf(3, entryCount);
    // // cout << entryCount << endl;

    // Read the input file
    ifstream cloudFile = ifstream(fname);
    int counter = 0;
    string line;
    while (getline(cloudFile, line)) {
        ++counter;
        if (counter < 12) continue; // Skip the first couple of lines
        
        istringstream iss(line);
        string entry;
        for (int i=0; i<3; ++i) {
            iss >> entry;
            result(i,counter-12) = stof(entry);
        }   
    }

    // // cout << result.block<3,10>(0,0) << endl;
    cloudFile.close();
    return result;
}

int countLines(const string& fname) {
    ifstream inFile = ifstream(fname);
    int counter = 0;
    string line;
    while (getline(inFile, line)) {
        ++counter;
    }
    inFile.close();
    return counter;
}

float getDepthAtRay(const Matrix<float, 3, Dynamic>& cloud, const vector<float>& depths, const Vector3f& ray) {
    Matrix<float, 1, Dynamic> scores = ray.transpose() * cloud;

    float depth = 1000;
    for (int i=0; i<scores.cols(); ++i) {
        if ((scores[i] > rayErrorLimit) && depths[i] < depth)
            depth = depths[i];
    }
    if (depth > 100) depth = 0.0;

    return depth;
}

vector<Vector3f> filterCloud(const Matrix<float,3,Dynamic>& cloud, const vector<Vector3f>& camRays) {
    // Get camrays min and max
    float minx = 1e8, maxx = -1e8, miny = 1e8, maxy = -1e8;
    for (const Vector3f& ray : camRays) {
        if (ray.x() < minx) minx = ray.x();
        else if (ray.x() > maxx) maxx = ray.x();
        if (ray.y() < miny) miny = ray.y();
        else if (ray.y() > maxy) maxy = ray.y();
    }

    // Filter the cloud
    vector<Vector3f> goodCloudVec;
    for (int i=0; i<cloud.cols();++i) {
        if (
            (cloud(0,i) >= (minx-rayEps)*cloud(2,i)) &&
            (cloud(0,i) <= (maxx+rayEps)*cloud(2,i)) &&
            (cloud(1,i) >= (miny-rayEps)*cloud(2,i)) &&
            (cloud(1,i) <= (maxy+rayEps)*cloud(2,i)) &&
            (cloud(2,i) > 0)
        ) {
            goodCloudVec.emplace_back(cloud.block<3,1>(0,i));
        }
    }

    return goodCloudVec;
}

vector<Vector3f> makeCameraRays() {

    // Make initial rays
    vector<Point2f> pixelRays(resx*resy);
    for (int x=0;x<resx;++x){
    for (int y=0;y<resy;++y){
        pixelRays[x*resy+y] = Point2f(x,y);
    }
    }

    // intrinsics [458.654, 457.296, 367.215, 248.375]
    float KData[9] = {458.654, 0, 367.215, 0, 457.296, 248.375, 0,       0,       1};
    Mat K0 = Mat(3,3, CV_32FC1, KData);
    K0 = K0 / skipPixels;
    K0.at<float>(2,2) = 1.0;
    const Mat K = K0;

    float distData[4] = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
    const Mat dist = Mat(1,4, CV_32FC1, distData);

    // undistort the rays
    vector<Point2f> normalRaysCV;
    undistortPoints(pixelRays, normalRaysCV, K, dist);

    // Turn into eigen
    auto rayLambda = [](const Point2f& nray) {
        Vector3f ray;
        ray << nray.x, nray.y, 1.0;
        ray.normalize();
        return ray;
    };
    vector<Vector3f> cameraRays(normalRaysCV.size());
    transform(normalRaysCV.begin(), normalRaysCV.end(), cameraRays.begin(), rayLambda);

    return cameraRays;
}

vector<float> makeDepthArray(const vector<Vector3f>& camRays, const vector<Vector3f>& goodCloudVec) {
    Matrix<float, 3, Dynamic> goodCloud = MatrixXf(3, goodCloudVec.size());
    vector<float> depths(goodCloudVec.size());
    for (int i=0; i<goodCloudVec.size();++i) {
        depths[i] = goodCloudVec[i].norm();
        goodCloud.block<3,1>(0,i) = goodCloudVec[i].normalized();
    }

    auto depthLambda = [&goodCloud, &depths] (const Vector3f& ray) { return getDepthAtRay(goodCloud, depths, ray); };
    vector<float> depthArray(camRays.size());
    transform(std::execution::par_unseq, camRays.begin(), camRays.end(), depthArray.begin(), depthLambda);
    // transform(camRays.begin(), camRays.end(), depthArray.begin(), depthLambda);

    // Set up sub array for testing
    // 274867
    // Matrix<float,3,Dynamic> subcloud = cloud.block<3,274867>(0,0);
    // auto depthLambda = [&subcloud, &depths] (const Vector3f& ray) { return getDepthAtRay(subcloud, depths, ray); };
    // transform(camRays.begin(), camRays.end(), depthArray.begin(), depthLambda);

    return depthArray;
}


vector<Pose> readCameraPoses(const string& fname) {
    const int numPoses = countLines(fname);
    vector<Pose> poses;

    // Read the input file
    ifstream poseFile = ifstream(fname);
    int counter = 0;
    string line;
    while (getline(poseFile, line)) {
        ++counter;
        if (counter < 2) continue; // Skip the header
        
        istringstream iss(line);
        string entry;
        vector<string> lineData;
        while (getline(iss, entry, ',')) {
            lineData.emplace_back(entry);
        }

        // Parse the line data
        Pose camPose;
        camPose.t = stoll(lineData[0]);
        camPose.x << stof(lineData[1]), stof(lineData[2]), stof(lineData[3]);
        Quaternionf q = Quaternionf(stof(lineData[4]), stof(lineData[5]), stof(lineData[6]), stof(lineData[7]));
        camPose.R = q.matrix();

        poses.emplace_back(camPose);
    }

    // // cout << result.block<3,10>(0,0) << endl;
    poseFile.close();
    return poses;
}

vector<long long> readStamps(const string& fname) {
    const int numStamps = countLines(fname);
    vector<long long> stamps;

    // Read the input file
    ifstream stampFile = ifstream(fname);
    int counter = 0;
    string line;
    while (getline(stampFile, line)) {
        ++counter;
        if (counter < 2) continue; // Skip the header
        
        istringstream iss(line);
        string entry;
        
        getline(iss, entry, ','); // Read just the first entry on the line
        stamps.emplace_back(stoll(entry));

        // cout << "Stamp " << stamps[counter-2] << endl;
    }

    // // cout << result.block<3,10>(0,0) << endl;
    stampFile.close();
    return stamps;
}