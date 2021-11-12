//
// Created by pbustos on 12/11/21.
//

#include "bug_navigation.h"

Bug_Navigation::Bug::Navigation()
{}

void Bug_Navigation::compute()
{
    static State_Base state = State_Base::IDLE;
    float advance = 0.0, rot = 0.0;

    auto tr = from_world_to_robot(target.to_eigen());
    auto dist = tr.norm();
    auto beta = atan2(tr.x(),tr.y());
    clear_path_to_point(QPointF(tr.x(), tr.y()), laser_poly);

    switch (state)
    {
        case State_Base::IDLE:
            if(target.active)
                state = State_Base::FORWARD;
            break;
        case State_Base::FORWARD:
        {
            qInfo() << __FUNCTION__ << "FORWARD";
            if(dist<100)
            {
                advance = 0; rot = 0;
                state = State_Base::IDLE;
                break;
            }
            if (min_laser_distance(ldata.size() / 2 - 10, ldata.size() / 2 + 10) < 800 and
                not clear_path_to_point(QPointF(tr.x(), tr.y()), laser_poly))
            {
                state = State_Base::TURN;
                advance = advance / 3;
                qInfo() << __FUNCTION__ << "cambio TURN";
                break;
            }
            // ----------------------------
            //rot = -(2.f / 100) * body_x_error;
            rot = beta;
            if (dist < 100)
                advance = 0.0;
            else
            {
                float dist_factor = std::clamp(dist / 1000.0, 0.0, 1.0);
                advance = robot.max_advance_speed * dist_factor * exp(-rot * rot * 2);
            }
            break;
        }
        case State_Base::TURN:
        {
            float dist_to_obs = min_laser_distance(ldata.size() / 2 - 10, ldata.size() / 2 + 10);
            qInfo() << __FUNCTION__ << "TURN: dist" << dist_to_obs;
            if (dist_to_obs > 1000)
            {
                state = State_Base::BORDER;
                qInfo() << __FUNCTION__ << " cambio BORDER";
                break;
            }
            rot = 0.8;
//            float dist_factor = std::clamp(dist / 1000.0, 0.0, 1.0);
//            advance = robot.max_advance_speed * dist_factor * exp(-rot * rot * 2);
            advance = 0.0;
            cout << ":::::::::::: TURN, rot: " << rot << " advance: " << advance << endl;
            break;
        }
        case State_Base::BORDER:
        {
            qInfo() << __FUNCTION__ << "BORDER";
            if (clear_path_to_point(QPointF(tr.x(), tr.y()), laser_poly))
            {
                state = State_Base::FORWARD;
                qInfo() << __FUNCTION__ << "cambio FORWARD";
                break;
            }
//            if (min_laser_distance(ldata.size() / 2 - 10, ldata.size() / 2 + 10) < 500)
//            {
//                state = State_Base::TURN;
//                qInfo() << __FUNCTION__ << "cambio TURN";
//                break;
//            }
            float lateral_distance = min_laser_distance(ldata.size()/2+20, ldata.size());
            if (lateral_distance < 300)
                rot = 1;
            else if (lateral_distance > 400)
                rot = -1;
            else
                rot = 0.0;
            float dist_factor = std::clamp(dist / 1000.0, 0.0, 1.0);
            advance = robot.max_advance_speed * dist_factor /** gaussian(rot)*/;
            break;
        }
    }

    const float gain = 0.5;
    qInfo() << __FUNCTION__ << "Send command:" << advance << rot;
//        differentialrobot_proxy->setSpeedBase(advance, gain * rot);
//        robot.current_rot_speed = gain*rot;
//        robot.current_adv_speed = advance;
   }