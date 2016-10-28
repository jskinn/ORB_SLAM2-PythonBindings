#include <opencv2/core/core.hpp>
#include <ORB_SLAM2/KeyFrame.h>
#include <ORB_SLAM2/Converter.h>
#include "ORBSlamPython.h"

BOOST_PYTHON_MODULE(orbslam2)
{
    boost::python::class_<ORBSlamPython, boost::noncopyable>("System", boost::python::init<std::string, std::string>())
        .def("initialize", &ORBSlamPython::initialize)
        .def("load_and_process_image", &ORBSlamPython::loadAndProcessImage)
        .def("shutdown", &ORBSlamPython::shutdown)
        .def("is_running", &ORBSlamPython::isRunning)
        .def("reset", &ORBSlamPython::reset)
        .def("get_trajectory_points", &ORBSlamPython::getTrajectoryPoints)
        .def("save_settings", &ORBSlamPython::saveSettings)
        .def("load_settings", &ORBSlamPython::loadSettings)
        .def("save_settings_file", &ORBSlamPython::saveSettingsFile)
        .staticmethod("save_settings_file")
        .def("load_settings_file", &ORBSlamPython::loadSettingsFile)
        .staticmethod("load_settings_file");
}

ORBSlamPython::ORBSlamPython(std::string vocabFile, std::string settingsFile)
    : vocabluaryFile(vocabFile),
    settingsFile(settingsFile),
    system(std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR))
{
    
}

ORBSlamPython::~ORBSlamPython()
{
}

bool ORBSlamPython::initialize()
{
    return system->StartUp(vocabluaryFile, settingsFile, false);
}

bool ORBSlamPython::isRunning()
{
    return system && system->IsRunning();
}

void ORBSlamPython::reset()
{
    if (system)
    {
        system->Reset();
    }
}

bool ORBSlamPython::loadAndProcessImage(std::string imageFile, double timestamp)
{
    cv::Mat im = cv::imread(imageFile, CV_LOAD_IMAGE_UNCHANGED);
    if (im.data) {
        cv::Mat pose = system->TrackMonocular(im, timestamp);
        return pose.empty();
    } else {
        return false;
    }
}

void ORBSlamPython::shutdown()
{
    system->Shutdown();
}

boost::python::list ORBSlamPython::getTrajectoryPoints() const
{
    // This is copied from the ORB_SLAM2 System.SaveTrajectoryTUM function, with some changes to output a python tuple.
    vector<std::shared_ptr<ORB_SLAM2::KeyFrame>> vpKFs = system->GetKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    boost::python::list trajectory;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        std::shared_ptr<ORB_SLAM2::KeyFrame> pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        
        trajectory.append(boost::python::make_tuple(
            pKF->mTimeStamp,    // Timestamp
            t.at<float>(0), t.at<float>(1), t.at<float>(2),  // Location
            q[0], q[1], q[2], q[3]  // Orientation
        ));

    }

    return trajectory;
}

bool ORBSlamPython::saveSettings(boost::python::dict settings) const
{
    return ORBSlamPython::saveSettingsFile(settings, settingsFile);
}

boost::python::dict ORBSlamPython::loadSettings() const
{
    return ORBSlamPython::loadSettingsFile(settingsFile);
}

bool ORBSlamPython::saveSettingsFile(boost::python::dict settings, std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::WRITE);
    
    boost::python::list keys = settings.keys();
    for (int index = 0; index < boost::python::len(keys); ++index)
    {
        boost::python::extract<std::string> extractedKey(keys[index]);
        if (!extractedKey.check())
        {
            continue;
        }
        std::string key = extractedKey;
        
        boost::python::extract<int> intValue(settings[key]);
        if (intValue.check())
        {
            fs << key << int(intValue);
            continue;
        }
        
        boost::python::extract<float> floatValue(settings[key]);
        if (floatValue.check())
        {
            fs << key << float(floatValue);
            continue;
        }
        
        boost::python::extract<std::string> stringValue(settings[key]);
        if (stringValue.check())
        {
            fs << key << std::string(stringValue);
            continue;
        }
    }
    
    return true;
}

// Helpers for reading cv::FileNode objects into python objects.
boost::python::list readSequence(cv::FileNode fn, int depth=10);
boost::python::dict readMap(cv::FileNode fn, int depth=10);

boost::python::dict ORBSlamPython::loadSettingsFile(std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::READ);
    return readMap(fs.getFirstTopLevelNode());
}


// ----------- HELPER DEFINITIONS -----------
boost::python::dict readMap(cv::FileNode fn, int depth)
{
    boost::python::dict map;
    if (fn.isMap()) {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it) {
            cv::FileNode item = *it;
            std::string key = item.name();
            
            if (item.isNone())
            {
                map[key] = boost::python::object();
            }
            else if (item.isInt())
            {
                map[key] = int(item);
            }
            else if (item.isString())
            {
                map[key] = std::string(item);
            }
            else if (item.isReal())
            {
                map[key] = float(item);
            }
            else if (item.isSeq() && depth > 0)
            {
                map[key] = readSequence(item, depth-1);
            }
            else if (item.isMap() && depth > 0)
            {
                map[key] = readMap(item, depth-1);  // Depth-limited recursive call to read inner maps
            }
        }
    }
    return map;
}

boost::python::list readSequence(cv::FileNode fn, int depth)
{
    boost::python::list sequence;
    if (fn.isMap()) {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it) {
            cv::FileNode item = *it;
            
            if (item.isNone())
            {
                sequence.append(boost::python::object());
            }
            else if (item.isInt())
            {
                sequence.append(int(item));
            }
            else if (item.isString())
            {
                sequence.append(std::string(item));
            }
            else if (item.isReal())
            {
                sequence.append(float(item));
            }
            else if (item.isSeq() && depth > 0)
            {
                sequence.append(readSequence(item, depth-1)); // Depth-limited recursive call to read nested sequences
            }
            else if (item.isMap() && depth > 0)
            {
                sequence.append(readMap(item, depth-1));
            }
        }
    }
    return sequence;
}