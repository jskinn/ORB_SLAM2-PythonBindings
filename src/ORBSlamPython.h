#ifndef ORBSLAMPYTHON_H
#define ORBSLAMPYTHON_H

#include <memory>
#include <Python.h>
#include <boost/python.hpp>
#include <ORB_SLAM2/System.h>

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
    boost::python::list getTrajectoryPoints() const;
    bool saveSettings(boost::python::dict settings);
    
private:
    std::string vocabluaryFile;
    std::string settingsFile;
    std::shared_ptr<ORB_SLAM2::System> system;
};



#endif // ORBSLAMPYTHON_H
