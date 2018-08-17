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
            res.warning << "Entity: '" << req.id << "' has the same pose and originalPose and thus was likely never state-updated. Please use /ed/kinect/state-update to update before using /ed/kinect/get_state."  << std::endl;
        }


        // xor the flags, only one can and has to be set
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
                        res.warning << "Entity: '" << req.id << "' as main object of group " << other->stateUpdateGroup() << " has no pose." << std::endl;
                        mainHasPoseAndOrigPose = false;
                    }

                    if(!other->has_original_pose())
                    {
                        res.warning << "Entity: '" << req.id << "' as main object of group " << other->stateUpdateGroup() << " has no original pose." << std::endl;
                        mainHasPoseAndOrigPose = false;
                    }

                    if(mainHasPoseAndOrigPose)
                    {
                        //Found main in thisObjects group
                        foundMain = true; // -> break loop
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
                double value, closeValue, openValue;

                if(thisEntity->stateDefinition()->angle)
                {
                    double origDif = ed_sensor_integration::math_helper::AngleBetweenTwoQuaternions(mainEntity->originalPose().getQuaternion(), mainEntity->pose().getQuaternion());
                    double currDif = ed_sensor_integration::math_helper::AngleBetweenTwoQuaternions(thisEntity->originalPose().getQuaternion(), thisEntity->pose().getQuaternion());

                    value = origDif - currDif;
                    value = ed_sensor_integration::math_helper::fmod(value+3.14,6.28)-3.14;

                    closeValue = thisEntity->stateDefinition()->angleDifferenceClose;
                    openValue = thisEntity->stateDefinition()->angleDifferenceOpen;
                }
                else
                {
                    const double yawDif = ed_sensor_integration::math_helper::AngleBetweenTwoQuaternions(thisEntity->originalPose().getQuaternion(),thisEntity->pose().getQuaternion());
                    double c = cos(yawDif);
                    double s = sin(yawDif);
                    geo::Mat3 yawRotMat = geo::Mat3(c, -s, 0, s, c, 0, 0, 0, 1);

                    // rotate orginal model position um main orginal position
                    geo::Vec3 orig_transformed = mainEntity->originalPose().getOrigin() - thisEntity->originalPose().getOrigin(); // move to 0,0
                    orig_transformed = yawRotMat * orig_transformed; // rotate like main object rotated
                    orig_transformed += thisEntity->originalPose().getOrigin(); // move back to mainobjet space origin

                    // move orginal model position mit curmain. curorg offset.
                    geo::Vec3 mainDiff = thisEntity->pose().getOrigin() - thisEntity->originalPose().getOrigin() ;
                    orig_transformed += mainDiff;

                    value = (mainEntity->pose().getOrigin()-orig_transformed).length();

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


                if(res.stateRatio < 0 || res.stateRatio > 1)
                {
                    res.warning << "Ratio is out of bounds (" << res.stateRatio << "). This is not critical, but you might check the YAML description of the entity." << std::endl;
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
