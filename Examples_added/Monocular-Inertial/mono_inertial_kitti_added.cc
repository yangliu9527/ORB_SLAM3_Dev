#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImages,
               vector<double> &vTimestamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./stereo_inertial_kitti path_to_vocabulary path_to_settings path_to_sequence_folder" << endl;
        return 1;
    }



    // Load all sequences:
    vector<string> vstrImages;
    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;
    int nImages;
    int nImu;
    int first_imu = 0;
    int tot_images = 0;

    cout << "Loading images for kitti sequence ...";

    string pathSeq(argv[3]);

    LoadImages(pathSeq, vstrImages, vTimestampsCam);
    cout << "LOADED!" << endl;

    cout << "Loading IMU for kitti sequence ...";
    LoadIMU(pathSeq, vTimestampsImu, vAcc, vGyro);
    cout << "LOADED!" << endl;

    nImages = vstrImages.size();
    nImu = vTimestampsImu.size();

    if((nImages<=0)||(nImu<=0))
    {
        cerr << "ERROR: Failed to load images or IMU for kitti sequence"  << endl;
        return 1;
    }

        // Find first imu to be considered, supposing imu measurements start first

    while(vTimestampsImu[first_imu]<=vTimestampsCam[0])
        first_imu++;
    first_imu--; // first imu measurement to be considered

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);

    cv::Mat imLeft, imRight;
    // for (seq = 0; seq<num_seq; seq++)
    // {
        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        double t_rect = 0.f;
        double t_resize = 0.f;
        double t_track = 0.f;
        int num_rect = 0;
        int proccIm = 0;
        for(int ni=0; ni<nImages; ni++, proccIm++)
        {
            // Read left and right images from file
            imLeft = cv::imread(vstrImages[ni],cv::IMREAD_UNCHANGED);

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImages[ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[ni];

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
                while(vTimestampsImu[first_imu]<=vTimestampsCam[ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x,vAcc[first_imu].y,vAcc[first_imu].z,
                                                             vGyro[first_imu].x,vGyro[first_imu].y,vGyro[first_imu].z,
                                                             vTimestampsImu[first_imu]));
                    first_imu++;
                }

    #if (defined COMPILEDWITHC11) || (defined COMPILEDWITHC14)
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the images to the SLAM system
            SLAM.TrackMonocular(imLeft,tframe,vImuMeas);

    #if (defined COMPILEDWITHC11) || (defined COMPILEDWITHC14)
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_rect + t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestampsCam[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }



    //}
    // Stop all threads
    SLAM.Shutdown();


    // Save camera trajectory
    // if (bFileName)
    // {
    //     const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
    //     const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
    //     SLAM.SaveTrajectoryEuRoC(f_file);
    //     SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    // }
    //else
    {
        SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
        //SLAM.SaveKeyFrameTrajectoryKITTI("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImages,
                vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_2/";

    const int nTimes = vTimestamps.size();
    vstrImages.resize(nTimes);


    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImages[i] = strPrefixLeft + ss.str() + ".png";
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{

    string ImuPath = strImuPath+"/imu.txt";
    ifstream fImu;
    fImu.open(ImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            double t, acc_x, acc_y, acc_z, angvel_x, angvel_y, angvel_z;
            stringstream ss;
            ss << s;
            ss >> t;
            ss >> acc_x;
            ss >> acc_y;
            ss >> acc_z;
            ss >> angvel_x;
            ss >> angvel_y;
            ss >> angvel_z;

            vTimeStamps.push_back(t);
            vAcc.push_back(cv::Point3f(acc_x, acc_y, acc_z));
            vGyro.push_back(cv::Point3f(angvel_x, angvel_y, angvel_z));
        }
    }
}
