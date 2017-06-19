#include <algorithm_testing/algorithmTesting.h> 

int main (int argc, char **argv) {    
    
    ros::init(argc, argv, "testing");
    
    algorithmTesting tracker;
    
    tracker.run();
    
    return 0;
}