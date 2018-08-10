#include "ed/kinect/recognizeState.h"

#include <ed/world_model.h>
#include <ed/entity.h>

#include "ed/kinect/math_helper.h"

// ----------------------------------------------------------------------------------------------------


RecognizeState::RecognizeState()
{
}

RecognizeState::~RecognizeState()
{
}


// ----------------------------------------------------------------------------------------------------

bool RecognizeState::recognizeState(const ed::WorldModel& world, const RecognizeStateRequest& req, RecognizeStateResult& res)
{
    ed::EntityConstPtr thisEntity = world.getEntity(req.id);
    bool hasError = false;

    if (!thisEntity)
    {
        res.error << "No such entity: '" << req.id << "'.";
        hasError = true;
    }
    else if (!thisEntity->has_state_definition())
    {
        res.error << "Entity: '" << req.id << "' has no state definition.";
        hasError = true;
    }
    else if (thisEntity->stateUpdateGroup().empty())
    {
        res.error << "Entity: '" << req.id << "' is not a member of any state update group and thus can not be positioned relative to the main object in the group.";
        hasError = true;
    }
    else if (!thisEntity->has_pose())
    {
        res.error << "Entity: '" << req.id << "' has no pose.";
        hasError = true;
    }
    else if (!thisEntity->has_original_pose())
    {
        res.error << "Entity: '" << req.id << "' has a pose but no original pose. This is strange and should not happen. Maybe its Beelzebots work.";
        hasError = true;
    }

    if(!hasError)
    {
        if(thisEntity->pose().t == thisEntity->originalPose().t && thisEntity->pose().R == thisEntity->originalPose().R)
        {
            res.warning << "Entity: '" << req.id << "' has the same pose and originalPose and thus was likely never state-updated. Please use /ed/kinect/state-update to update before using /ed/kinect/get_state.";
        }



        if(thisEntity->stateDefinition()->angle != thisEntity->stateDefinition()->position)
        {
            std::string groupName = thisEntity->stateUpdateGroup();
            bool foundMain = false;
            bool someMainsHadNoPoseOrOriginalPose = false;
            ed::EntityConstPtr mainEntity;

            for(ed::WorldModel::const_iterator it = world.begin(); it != world.end() && foundMain == false; ++it)
            {
                const ed::EntityConstPtr& other = *it;

                if(groupName.compare(other->stateUpdateGroup()) == 0 && other->hasFlag("state-update-group-main"))
                {
                    bool mainHasPoseAndOrigPose = true;
                    if(!other->has_pose())
                    {
                        res.warning << "Entity: '" << req.id << "' as main object of group " << other->stateUpdateGroup() << " has no pose.";
                        mainHasPoseAndOrigPose = false;
                    }

                    if(!other->has_original_pose())
                    {
                        res.warning << "Entity: '" << req.id << "' as main object of group " << other->stateUpdateGroup() << " has no original pose.";
                        mainHasPoseAndOrigPose = false;
                    }

                    if(mainHasPoseAndOrigPose)
                    {
                        //Found main in thisObjects group
                        foundMain = true;
                        mainEntity = other;
                    }
                    else
                    {
                        someMainsHadNoPoseOrOriginalPose = true;
                    }
                }
            }

            if(!foundMain)
            {
                if(someMainsHadNoPoseOrOriginalPose)
                {
                    res.error << "No main object with pose and/or original pose found in group " << thisEntity->stateUpdateGroup() << ".";
                }
                else
                {
                    res.error << "No main object found in group " << thisEntity->stateUpdateGroup() << ".";
                }
                hasError = true;
            }
            else
            {


                      // bool angle;
                      // bool position;
                      // float angleDifferenceClose;
                      // float angleDifferenceOpen;
                      // float positionDifferenceClose;
                      // float positionDifferenceOpen;

                double value, closeValue, openValue;

                if(thisEntity->stateDefinition()->angle)
                {
                    //TODO: Not tested and check the math
                    // double origMainAngle = ed_sensor_integration::math_helper::QuaternionToYaw(mainEntity->originalPose().getQuaternion());
                    // double origThisAngle = ed_sensor_integration::math_helper::QuaternionToYaw(thisEntity->originalPose().getQuaternion());
                    //
                    // double currMainAngle = ed_sensor_integration::math_helper::QuaternionToYaw(mainEntity->pose().getQuaternion());
                    // double currThisAngle = ed_sensor_integration::math_helper::QuaternionToYaw(thisEntity->pose().getQuaternion());
                    //
                    // double origDif = origThisAngle - origMainAngle;
                    // double currDif = currThisAngle - currMainAngle;
                    //
                    // value = currDif - origDif;
                    //
                    // closeValue = thisEntity->stateDefinition()->angleDifferenceClose;
                    // openValue = thisEntity->stateDefinition()->angleDifferenceOpen;
                }
                else
                {
                    //TODO: Check the math
                    double mainDif = (mainEntity->originalPose().getOrigin() - mainEntity->pose().getOrigin()).length();
                    double thisDif = (thisEntity->originalPose().getOrigin() - thisEntity->pose().getOrigin()).length();

                    value = thisDif - mainDif;

                    closeValue = thisEntity->stateDefinition()->positionDifferenceClose;
                    openValue = thisEntity->stateDefinition()->positionDifferenceOpen;
                }

                res.stateRatio = (value - closeValue) / (openValue - closeValue);

                if(res.stateRatio > .25)
                {
                    res.state = "open";
                }
                else
                {
                    res.state = "closed";
                }
            }
        }
        else
        {
            res.error << "State definition of " << req.id << " either uses both angle and position or neither. Only one of both has to be used in the YAML definition.";
            hasError = true;
        }
    }

    if(hasError)
    {
        res.state = "";
        res.stateRatio = -1;
    }

    return !hasError;
}
