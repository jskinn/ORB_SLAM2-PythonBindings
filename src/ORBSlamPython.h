#ifndef ORBSLAMPYTHON_H
#define ORBSLAMPYTHON_H

#include <memory>
#include <Python.h>
#include <boost/python.hpp>
#include <ORB_SLAM2/System.h>
#include <ORB_SLAM2/Tracking.h>

class ORBSlamPython
{
public:
    ORBSlamPython(std::string vocabFile, std::string settingsFile);
    ~ORBSlamPython();
    
    bool initialize();
    bool isRunning();
    bool loadAndProcessImage(std::string imageFile, double timestamp);
    void reset();
    void shutdown();
    ORB_SLAM2::Tracking::eTrackingState getTrackingState() const;
    boost::python::list getTrajectoryPoints() const;
    bool saveSettings(boost::python::dict settings) const;
    boost::python::dict loadSettings() const;
    void setResolution(int x, int y);
    
    static bool saveSettingsFile(boost::python::dict settings, std::string settingsFilename);
    static boost::python::dict loadSettingsFile(std::string settingsFilename);
    
private:
    std::string vocabluaryFile;
    std::string settingsFile;
    std::shared_ptr<ORB_SLAM2::System> system;
    int resolutionX;
    int resolutionY;
};



#endif // ORBSLAMPYTHON_H
