#ifndef ED_KINECT_RECOGNIZE_STATE_H_
#define ED_KINECT_RECOGNIZE_STATE_H_

#include <string>
#include <sstream>

#include <ed/world_model.h>

// ----------------------------------------------------------------------------------------------------

struct RecognizeStateRequest
{
    RecognizeStateRequest(std::string id) : id(id) {}

    std::string id;
};

// ----------------------------------------------------------------------------------------------------

struct RecognizeStateResult
{
    RecognizeStateResult() : state(""), stateRatio(-1) {}

    std::string state;
    float stateRatio;
    std::stringstream error;
    std::stringstream warning;
};

// ----------------------------------------------------------------------------------------------------

class RecognizeState
{

public:

    RecognizeState();

    ~RecognizeState();

    bool recognizeState(const ed::WorldModel& world, const RecognizeStateRequest& req, RecognizeStateResult& res);
};

#endif
