#include <memory>
#include "ORBSlamPython.h"
//#include <gtest/gtest.h>

// TODO: Actually use google test, copy the example here: https://github.com/snikulov/google-test-examples
/*TEST(SystemTest, testAllocateAndDeallocate) {
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>("Vocabluary/ORBvoc.txt", "Examples/Monocular/TUM1.yaml");
}*/

int main(int argc, char** argv)
{
    {
        //testing::InitGoogleTest(&argc, argv);
        //return RUN_ALL_TESTS();
        std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    
    }
    
    {
        std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
        subject->StartUp(argv[1], argv[2], false);
        subject->Shutdown();
    }
    
    {
        std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
        subject->StartUp(argv[1], argv[2], true);
        subject->Shutdown();
    }
}
