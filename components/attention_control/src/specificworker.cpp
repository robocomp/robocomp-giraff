/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    try
    {
        RoboCompCommonBehavior::Parameter par = params.at("inverted_tilt");
        if(par.value == "true" or par.value == "True")
            inverted_tilt = -1;
        else inverted_tilt = 1;
    }
    catch(const std::exception &e) { qFatal("Error reading config params"); }
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;

    try{ jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{0.0001, 1 });}  // radians. 0 vertical
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    // Load YOLO network
    std::string modelConfiguration = "yolov3/yolov3.cfg";  //yolov4 is not supported in this version of OPenCV
    std::string modelWeights = "yolov3/yolov3.weights";
    std::string classesFile = "yolov3/coco.names";
    std::ifstream ifs(classesFile.c_str());
    std::string line;
    while (getline(ifs, line)) classes.push_back(line);
    net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    // Viewer
    auto dimensions = QRectF(-5000, -2500, 10000, 5000);
    viewer = new AbstractGraphicViewer(this, dimensions);
    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract

    // state machines
    connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));
    connect(&timer_l1, SIGNAL(timeout()), this, SLOT(compute_L1()));
    connect(&timer_l2, SIGNAL(timeout()), this, SLOT(compute_L2()));
    connect(&timer_l3, SIGNAL(timeout()), this, SLOT(compute_L3()));
    connect(&timer_bill, SIGNAL(timeout()), this, SLOT(compute_bill()));
    this->Period = 100;
    int Period_L1 = 80;
    int Period_L2 = 150;
    int Period_L3 = 220;
    int Period_Bill = 1000;
    if(this->startup_check_flag)
        this->startup_check();
    else
    {
        timer.start(Period);
        timer_l1.start(Period_L1);
        timer_l2.start(Period_L2);
        timer_l3.start(Period_L3);
        timer_bill.start(Period_Bill);
    }

    // send Bill to robot
    try
    {
        auto r_state = fullposeestimation_proxy->getFullPoseEuler();
        auto pose = billcoppelia_proxy->getPose();
        QLineF line(pose.x, pose.y, r_state.x, r_state.y);
        float percent = (line.length() - 600)/line.length();
        QPointF p = line.pointAt(percent);
        qInfo() << __FUNCTION__ << pose.x << pose.y << line << percent;
        billcoppelia_proxy->setTarget(p.x(), p.y());
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;};
}

////////////////////////////////////////////////////
// THREE LEVELS ARCH. EACH LEVEL IS A STATE MACHINE
///////////////////////////////////////////////////

// FIST LEVEL. read sensors and use classifiers to create dynamic control loops
// that keep perceived objects (person parts) centered (in focus). Output presence state and pose
void SpecificWorker::compute_L1()
{
    const float XMARK = 10;  // number or hits before getting 0.7
    const float YMARK = 0.7; // threshold value for next level
    static int cont = 0;
    const double s = -XMARK/(log(1.0/YMARK - 1.0));
    auto integrator = [s](double x){return 1.0/(1.0 + exp(-x/s));};

    const auto detected = read_image();
    const auto &[body_o, face_o] = detected;
    cont = std::clamp(cont, -20, 20);
    switch(l1_state)
    {
        case L1_State::SEARCHING:
            if (face_o.has_value())
            {
                auto &[_, __, d, center] = face_o.value();
                this->l1_person.pos = Eigen::Vector2f(0, d);
                this->l1_person.looking_at = Parts::FACE;
                l1_state = L1_State::FACE_DETECTED;
                return;
            } else if (body_o.has_value())
            {
                auto &[_, __, d, center] = body_o.value();
                l1_state = L1_State::BODY_DETECTED;
                this->l1_person.pos = Eigen::Vector2f(0, d);
                this->l1_person.looking_at = Parts::BODY;
                try{ differentialrobot_proxy->setSpeedBase(0, 0); robot.current_rot_speed = 0;  robot.current_adv_speed=0.0;}
                catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to differential robot" << std::endl; }
                return;
            }
            this->l1_person.looking_at = Parts::NONE;
            this->l1_person.dyn_state = integrator(cont--);
            // rotate to look for the person
            if( robot.current_rot_speed > 0)
                try{ differentialrobot_proxy->setSpeedBase(0, 0.7); robot.current_rot_speed = 0.7;  robot.current_adv_speed=0.0;}
                catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to differential robot" << std::endl; }
            else
                try{ differentialrobot_proxy->setSpeedBase(0, -0.7); robot.current_rot_speed = -0.7;  robot.current_adv_speed=0.0;}
                catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to differential robot" << std::endl; }
            break;

        case L1_State::BODY_DETECTED:
            if(not body_o.has_value())
            {
                l1_state = L1_State::SEARCHING;
                return;
            }
            move_tablet(detected);
            move_base(detected);
            this->l1_person.pos = Eigen::Vector2f(0, std::get<2>(body_o.value()));
            this->l1_person.dyn_state = integrator(cont++);
            break;

        case L1_State::FACE_DETECTED:
            if(not face_o.has_value())
            {
                //try{ emotionalmotor_proxy->isanybodythere(false);}
                //catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to emotional motor " << std::endl; }
                l1_state = L1_State::SEARCHING;
                return;
            }
            //try{ emotionalmotor_proxy->isanybodythere(true);}
            //catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to emotional motor " << std::endl; }
            this->l1_person.pos = Eigen::Vector2f(0, std::get<2>(face_o.value()));
            move_tablet(detected);
            move_base(detected);
            //move_eyes(face_o);
            this->l1_person.dyn_state = integrator(cont++);
            break;
        case L1_State::EYES_DETECTED:
            break;
        case L1_State::HANDS_DETECTED:
            break;
    }
   //this->l1_person.print();
}
void SpecificWorker::compute_L2()
{
    // SECOND LEVEL: belief function to create and maintain a represented person
    // we need here to create, maintain and destroy (when needed) a "represented person" that exists because
    // enough evidence supports it. We want evidence coming from the control loops, not just the classifiers
    // The logic here is, when new evidence (presence state and pose) gets in
    // if there is no person, create it. It must have certain internal structure.
    // from here, synthesize the evidence that should be coming and is relevant to the task
    // if evidence comes, check if it matches. If it does, update representation (and dynamic model of it)
    // if it does not match, note as counter evidence
    // it no evidence comes, note as counter evidence
    // if enough counter evidence has accumulated, destroy the representation

    switch (l2_state)                           /// ADD DYNAMIC STATES APPROACHING, etc and speed values
    {
        case L2_State::EXPECTING:
            if(this->l1_person.dyn_state > 0.7)
            {
                // create person
                L2_Person p;
                p.pos =  Eigen::Vector2f(0,0);
                p.vel = Eigen::Vector2f(0,0);
                p.looking_at = l1_person.looking_at;
                p.height = 160;
                p.name = "Bill";
                p.orientation = 0;
                p.angular_spped = 0;
                p.creation_time = std::chrono::system_clock::now();
                l2_people.push_back(p);
                l2_state = L2_State::PERSON;
            }
            break;
        case L2_State::PERSON:
            if(this->l1_person.dyn_state < 0.7)
            {
                // delete person. Maybe push into memory
                l2_people.clear();
                l2_state = L2_State::EXPECTING;
            }
            // update person
            L2_Person &p = l2_people[0];
            if(l1_person.pos.y() > 0 )  // don't update if bad depth reading
                p.pos = l1_person.pos;
            p.looking_at = l1_person.looking_at;
            // downward flow
            // project where person is going to be
            // plan perception annd send downwards anticipated proposal
            l1_person.current_action = p.current_action;
            break;
    }
//    if(not l2_people.empty())
//        l2_people.front().print();
}
void SpecificWorker::compute_L3()
{
// THIRD LEVEL.  Mission specific control program to decide what to do next based on the represented person
    static std::chrono::system_clock::time_point ready_start;
    static std::chrono::system_clock::time_point searching_initial_time;
    switch(l3_state)
    {
        case L3_State::WAITING:
            // if person detected move to PERSON_DETECTED
            if( not l2_people.empty()
                and l2_people.front().looking_at == Parts::FACE
                and l2_people.front().pos.norm() < 800)
            {
               l3_state = L3_State::READY_TO_INTERACT;
               ready_start = std::chrono::system_clock::now();
            }
            break;
        case L3_State::READY_TO_INTERACT:
            // Wait for a signal or order and move to FOLLOW
            if(std::chrono::duration_cast<chrono::seconds>(std::chrono::system_clock::now()-ready_start).count() > 1)
                l3_state = L3_State::START_FOLLOWING;
            break;
        case L3_State::START_FOLLOWING:
            // if no person go to searching
            if(l2_people.empty())
            {
                l3_state = L3_State::SEARCHING;
                searching_initial_time = std::chrono::system_clock::now();
                return;
            }
            // if there is person and a body go to following
            if( not l2_people.empty()
                and l2_people.front().looking_at == Parts::BODY)
                //and is moving away (velocity is > 0)
            {
                l3_state = L3_State::FOLLOWING;
            }
            break;
        case L3_State::FOLLOWING:
            // distance control
            if(l2_people.empty())
            {
                l3_state = L3_State::START_FOLLOWING;
                return;
            }
            l2_people[0].current_action = Actions::FOLLOW;
            break;
        case L3_State::SEARCHING:
            // person is lost. Wait for L2 and L2 levels to find it
            if(not l2_people.empty())
            {
                l3_state = L3_State::FOLLOWING;
                return;
            }
            if(std::chrono::duration_cast<chrono::seconds>(std::chrono::system_clock::now() - searching_initial_time).count() > 10)
            {
                l3_state = L3_State::WAITING;
            }
            break;
        case L3_State::STOP:
            break;
    };
    l3_person.print(l3_state);
}
void SpecificWorker::compute_bill()
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> x_random(-4500, 4500);
    static std::uniform_real_distribution<double> y_random(-2000, 4000);
    static Eigen::Vector2f bill_target;
    static std::vector<Eigen::Vector2f> points {Eigen::Vector2f(0, -1800),
                                                Eigen::Vector2f(-4000, -1800),
                                                Eigen::Vector2f(-4000, 1800),
                                                Eigen::Vector2f(4000, 1800),
                                                Eigen::Vector2f(4000, -1800),
                                                Eigen::Vector2f(0, -1800)};
    static std::vector<Eigen::Vector2f>::iterator iter = points.begin();

    switch (bill_state)
    {
        case Bill_State::NEW_POINT:
            qInfo() << __FUNCTION__ << "Bill - new point";
            if(l3_state == L3_State::START_FOLLOWING or l3_state == L3_State::FOLLOWING)
                // select new point
                try
                {
                    //bill_target = Eigen::Vector2f(x_random(mt), y_random(mt));
                    bill_target = *iter;
                    if(++iter==points.end()) iter = points.begin();
                    billcoppelia_proxy->setTarget(bill_target.x(), bill_target.y());
                    bill_state = Bill_State::TO_POINT;
                }
                catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;};
            break;
        case Bill_State::TO_POINT:
            qInfo() << __FUNCTION__ << "Bill - to_point";
            try
            {
                auto pose = billcoppelia_proxy->getPose();
                qInfo() << __FUNCTION__ << pose.vx << pose.vy;
                if(fabs(pose.vx) < 20 and fabs(pose.vy) < 20 )
                {
                    bill_state = Bill_State::NEW_POINT;
                }
            }
            catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;};
            break;
    }
}
void SpecificWorker::compute()
{
    try
    {
        ldata = laser_proxy->getLaserData();
        draw_laser( ldata );
    }
    catch(const Ice::Exception &e)
    { std::cout << e.what() << std::endl;}
    try
    {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
};

////////////////////////////////////////////////////////////////////
SpecificWorker::DetectRes SpecificWorker::read_image()
{
    RoboCompCameraSimple::TImage tablet_rgb;
    RoboCompCameraRGBDSimple::TRGBD top_rgbd;
    DetectRes ret{{},{}};
    try
    {
        top_rgbd = camerargbdsimple_proxy->getAll("camera_top");
        tablet_rgb = camerasimple_proxy->getImage();
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e.what() << " Empty image or comms problem" << std::endl;
        return ret;
    }
    const auto &top_image = top_rgbd.image;
    cv::Mat top_rgb_mat = cv::Mat(top_image.height, top_image.width, CV_8UC3, &top_rgbd.image.image[0]);
    const auto &top_depth = top_rgbd.depth;
    cv::Mat top_depth_mat = cv::Mat(top_depth.height, top_depth.width, CV_32F, &top_rgbd.depth.depth[0]);

    const int width = 300;  // YOLO INPUT
    const int height = 300;

    camera.cols = top_image.width;
    camera.rows = top_image.height;
    camera.focal_x = top_image.focalx;

    if (top_image.width != 0 and top_image.height != 0)
    {
//        cv::Mat img;
//        if (tablet_rgb.compressed)
//        {
//            auto img_uncomp = cv::imdecode(tablet_rgb.image, -1);
//            img = cv::Mat(tablet_rgb.height, tablet_rgb.width, CV_8UC3, &img_uncomp.data);
//        }
//        else
//            img = cv::Mat(tablet_rgb.height, tablet_rgb.width, CV_8UC3, &tablet_rgb.image[0]);

        // from RGBD camera

        cv::Mat n_img;
        cv::resize(top_rgb_mat, n_img, cv::Size(width, height));

        // face
        auto rectangles = face_detector.detect_face_rectangles(n_img);
        if (not rectangles.empty())
        {
            auto r = rectangles.front();
            cv::rectangle(n_img, r, cv::Scalar(0, 105, 205), 3);
            //qInfo() << __FUNCTION__ << "face " << rect;
            float x_scale = (float)(top_depth_mat.cols-1)/n_img.cols;
            float y_scale = (float)(top_depth_mat.rows-1)/n_img.rows;
            QRectF rect = QRectF( x_scale*r.x, y_scale*r.y, x_scale*r.width, y_scale*r.height);
            //qInfo() << __FUNCTION__ << "face " << rect << top_depth_mat.cols << top_depth_mat.rows;
            if(x_scale*r.x + x_scale*r.width > top_depth_mat.cols) rect.setWidth(top_depth_mat.cols-x_scale*r.x);
            if(y_scale*r.y + y_scale*r.height > top_depth_mat.rows) rect.setHeight(top_depth_mat.rows-y_scale*r.y);
            try
            {
                cv::Mat roi = top_depth_mat(cv::Rect((int) rect.x(), (int) rect.y(), (int) rect.width(), (int) rect.height()));
                float k=-1;
                if(roi.cols > 0 and roi.rows >0)
                {
                    double minVal;
                    double maxVal;
                    cv::Point minLoc;
                    cv::Point maxLoc;
                    cv::minMaxLoc(roi, &minVal, &maxVal, &minLoc, &maxLoc);
                    k = minVal * 1000; //mm
                }
                std::get<1>(ret) = {(top_depth_mat.cols/2.0) - rect.center().x(), (top_depth_mat.rows/2.0) - rect.center().y(), k, rect.center()};
            }
            catch(const std::exception &e)
            {
                std::cout << e.what() << std::endl;
                qInfo() << __FUNCTION__ << "body " << (int)rect.x() << (int)rect.y() << (int)rect.width() << (int)rect.height();
            };
        }
        else //body
        {
            // YOLO
            auto people = yolo_detector(n_img);
            if (not people.empty())
            {
                auto r = people.front();
                cv::rectangle(n_img, r, cv::Scalar(0, 0, 255), 3);
                float x_scale = (float) top_depth_mat.cols / n_img.cols;
                float y_scale = (float) top_depth_mat.rows / n_img.rows;
                QRectF rect = QRectF(x_scale * r.x, y_scale * r.y, x_scale * r.width, y_scale * r.height);
                if (x_scale * r.x + x_scale * r.width > top_depth_mat.cols) rect.setWidth(top_depth_mat.cols - x_scale * r.x - 1);
                if (y_scale * r.y + y_scale * r.height > top_depth_mat.rows) rect.setHeight(top_depth_mat.rows - y_scale * r.y - 1);
                try
                {
                    cv::Mat roi = top_depth_mat(cv::Rect((int) rect.x(), (int) rect.y(), (int) rect.width(), (int) rect.height()));
                    float k=-1;
                    if(roi.cols > 0 and roi.rows >0)
                    {
                        double minVal;
                        double maxVal;
                        cv::Point minLoc;
                        cv::Point maxLoc;
                        cv::minMaxLoc(roi, &minVal, &maxVal, &minLoc, &maxLoc);
                        k = minVal * 1000; //mm
                    }
                    std::get<0>(ret) = {(top_depth_mat.cols/2.0) - rect.center().x(), (top_depth_mat.rows/2.0) - rect.center().y(), k, rect.center()};
                }
                catch(const std::exception &e)
                {
                    std::cout << e.what() << std::endl;
                    qInfo() << __FUNCTION__ << "body " << (int)rect.x() << (int)rect.y() << (int)rect.width() << (int)rect.height();
                };
            }
        }
        // show image
        cv::cvtColor(n_img, n_img, cv::COLOR_BGR2RGB);
        cv::imshow("Camera tablet", n_img);
        cv::waitKey(1);
        return ret;
    }
    else
    {
        qWarning() << "Empty tablet image";
        return ret;
    }
}
void SpecificWorker::move_tablet(const DetectRes &detected)
{
    float tilt = 0.7;
    float pos = 0;
    const auto &[body_o, face_o] = detected;
    if(body_o.has_value())
    {
        auto[_, body_y_error, __, ___] = body_o.value();
        const float delta = 0.1;
        tilt = inverted_tilt * (delta / 100) * body_y_error;  // map from -100,100 to -0.1,0.1 rads
        try
        {
            pos = jointmotorsimple_proxy->getMotorState("tablet_joint").pos;
            //qInfo() << __FUNCTION__ << "BODY: pos" << pos << "error" << body_y_error << "tilt" << tilt << pos - tilt;
            jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1});
        }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to join motor " << std::endl; }
    }
    else if( face_o.has_value())
    {
        auto[_, face_y_error, __, ___] = face_o.value();
        const float delta = 0.1;
        tilt = inverted_tilt * (delta / 100) * face_y_error;  // map from -100,100 to -0.1,0.1 rads
        try
        {
            pos = jointmotorsimple_proxy->getMotorState("tablet_joint").pos;
            // qInfo() << __FUNCTION__ << "FACE: pos" << pos << "error" << face_y_error << "tilt" << tilt << pos - tilt;
            if(fabs(pos-tilt) > 0.17)
                jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1});
        } // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to join motor" << std::endl; }
    }
//    if (abs(tilt) > 0.1 )
//    {
//        try { jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1}); }  // radians. 0 vertical
//        catch (const Ice::Exception &e) { std::cout << e.what() <<  " No connection to join motor" << std::endl; }
//    }
}
void SpecificWorker::move_base(const DetectRes &detected)
{
    // rotate base
    float advance = 0.0;
    float rot = 0.0;
    const float MAX_ADVANCE_SPEED = 500;
    const float MIN_PERSON_DISTANCE = 700;
    const auto &[body_o, face_o] = detected;

    static QGraphicsItem *graphics_line = nullptr;
    if (graphics_line != nullptr)
        viewer->scene.removeItem(graphics_line);

    if(body_o.has_value())
    {
        auto &[body_x_error, _, body_dist, center] = body_o.value();
        //if person outside laser, compute a virtual waypoint inside laser and closest to person and closest to last virtualwaypoint
        if( const auto &[out, dist, angle] = person_outside_laser(center, body_dist); out == true)
        {
            QLineF line(toQPointF(from_robot_to_world(Eigen::Vector2f(0,0))),
                        toQPointF(from_robot_to_world(Eigen::Vector2f(dist*sin(angle),dist*cos(angle)))));
            //qInfo() << __FUNCTION__ << "Person not in sight" << dist << angle << line;
            graphics_line = viewer->scene.addLine(line, QPen(QColor("Orange"), 50));
            //rot = angle*0.9;
        }
       else
            rot = -(2.f / 100) * body_x_error;

        if(l1_person.current_action == Actions::FOLLOW)
        {
            if(fabs(body_dist-MIN_PERSON_DISTANCE) < 100)
                advance = 0;
            if((body_dist-MIN_PERSON_DISTANCE) < -300)
                advance = std::clamp(body_dist-MIN_PERSON_DISTANCE, -300.f, 0.f);
            else
                advance = MAX_ADVANCE_SPEED * (body_dist-MIN_PERSON_DISTANCE)*(1/1000.f) * exp(-rot*rot*2);
        }
//        if(body_dist > 0 and body_dist < 400)
//            advance = -(400.0 / 400.0) * body_dist;

    }
    else if(face_o.has_value())
    {
        auto &[face_x_error, _, face_dist, __] = face_o.value();
        rot = -(2.f / 100) * face_x_error;
//        if(face_dist > 0 and face_dist < 400)
//            advance = -(400.0 / 400.0) * face_dist;
    }

    //if (abs(rot) > 0.5 )  // avoid sending small velocities
    {
        try
        {
            const float gain = 0.5;
            differentialrobot_proxy->setSpeedBase(advance, gain * rot);
            robot.current_rot_speed = gain*rot;
            robot.current_adv_speed = advance;
        }
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }
}
std::tuple<bool, float, float> SpecificWorker::person_outside_laser(const QPointF &body, float body_dist)
{
    static float angle_ant = 0.0;

    QPolygonF laser_poly;
    laser_poly << QPointF(0,0);
    for(auto &&l : ldata)
        laser_poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
    laser_poly.pop_back();

    // body must be backprojected to XYZ camera coordinate system
    float bpy = body_dist;
    float bpx = (body.x()-camera.cols/2.0) * bpy / camera.focal_x;

    if(not clear_path_to_point(QPointF(bpx, bpy), laser_poly))
    {
        const float A=0.4, B=0.5, C=0.5, D=0.6;
        std::vector<std::tuple<float, float, float>> scores;
        for (const auto &l: ldata)
        {
            float x= l.dist * sin(l.angle); float y= l.dist * cos(l.angle);
            if (clear_path_to_point(QPointF(x,y), laser_poly))
           {
                float inv_dist_to_robot;
                if (l.dist > 0) inv_dist_to_robot = 1.0 / l.dist; else inv_dist_to_robot = std::numeric_limits<float>::max();
                float angle_to_robot = l.angle;
                float angle_to_previous = fabs(angle_ant - l.angle);
                float dist_to_person = (Eigen::Vector2f(bpx, bpy) - Eigen::Vector2f(x,y)).norm();
                float score = A * inv_dist_to_robot + B * angle_to_robot + C * angle_to_previous + D * dist_to_person;
                scores.push_back(std::make_tuple(score, l.dist, l.angle));
                // BUSCAR UNA CONDICION QUE EXCLUYA angulos entre el robot y la persona
            }
        }
        if( auto min = std::ranges::min_element(scores, [](auto a, auto b) { return std::get<0>(a) < std::get<0>(b); }); min != scores.end())
        {
            angle_ant = std::get<2>(*min);
            return std::make_tuple(true, std::get<1>(*min), std::get<2>(*min));
        }
    }
    return std::make_tuple(false, 0, 0);
}
bool SpecificWorker::clear_path_to_point(const QPointF &goal, const QPolygonF &laser_poly)
{
    //qInfo() << __FUNCTION__ << goal;

    // create tube lines
    Eigen::Vector2f goal_r(goal.x(), goal.y());
    Eigen::Vector2f robot(0.0,0.0);
    // number of parts the target vector is divided into
    float parts = (goal_r).norm()/(this->robot.length/4);
    Eigen::Vector2f rside(260, 200);
    Eigen::Vector2f lside(-260, 200);
    if(parts < 1) return true;

    bool res = true;
    QPointF p, q, r;
    // reduce length by size of person
    float limit = (this->robot.length)/(goal_r).norm();
    for(auto l: iter::range(0.0, 1.0-limit, 1.0/parts))
    {
        p = toQPointF(robot*(1-l) + goal_r*l);
        q = toQPointF((robot+rside)*(1-l) + (goal_r+rside)*l);
        r = toQPointF((robot+lside)*(1-l) + (goal_r+lside)*l);
        if( not laser_poly.containsPoint(p, Qt::OddEvenFill) or
            not laser_poly.containsPoint(q, Qt::OddEvenFill) or
            not laser_poly.containsPoint(r, Qt::OddEvenFill))
        {
            //qInfo() << __FUNCTION__<< goal << p << q << r;
            res = false;
            break;
        }
    }

    // draw
//    QLineF line_center(toQPointF(from_robot_to_world(robot)), toQPointF(from_robot_to_world(Eigen::Vector2f(p.x(),p.y()))));
//    QLineF line_right(toQPointF(from_robot_to_world(robot+rside)), toQPointF(from_robot_to_world(Eigen::Vector2f(q.x(),q.y()))));
//    QLineF line_left(toQPointF(from_robot_to_world(robot+lside)), toQPointF(from_robot_to_world(Eigen::Vector2f(r.x(),q.y()))));
//    static QGraphicsItem *graphics_line_center = nullptr;
//    static QGraphicsItem *graphics_line_right = nullptr;
//    static QGraphicsItem *graphics_line_left = nullptr;
//    static QGraphicsItem *graphics_target = nullptr;
//    if (graphics_line_center != nullptr)
//        viewer->scene.removeItem(graphics_line_center);
//    if (graphics_line_right != nullptr)
//        viewer->scene.removeItem(graphics_line_right);
//    if (graphics_line_left != nullptr)
//        viewer->scene.removeItem(graphics_line_left);
//    if (graphics_target != nullptr)
//        viewer->scene.removeItem(graphics_target);
//    graphics_line_center = viewer->scene.addLine(line_center, QPen(QColor("Blue"), 30));
//    graphics_line_right = viewer->scene.addLine(line_right, QPen(QColor("Orange"), 30));
//    graphics_line_left = viewer->scene.addLine(line_left, QPen(QColor("Magenta"), 30));
//    graphics_target = viewer->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("Blue")), QBrush(QColor("Blue")));
//    auto goal_w = from_robot_to_world(goal_r);
//    graphics_target->setPos(goal_w.x(), goal_w.y());

    return res;
}
void SpecificWorker::move_eyes(std::optional<std::tuple<int,int,int>> face_o)
{
    if( face_o.has_value())
    {

        auto [face_x_error, face_y_error, __] = face_o.value();
        const float delta = 0.1;
        const float tilt_x = (delta / 10) * face_x_error;  // map from -100,100 to -0.1,0.1 rads
        const float tilt_y = -(delta / 10) * face_y_error;  // map from -100,100 to -0.1,0.1 rads
        //qInfo() << __FUNCTION__ << "FACE: pos" << pos << "error" << face_y_error << "tilt" << tilt << pos - tilt;
        try { emotionalmotor_proxy->pupposition(tilt_x,tilt_y); }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to emotional motor" << std::endl; }
    }
}
std::vector<cv::Rect> SpecificWorker::yolo_detector(cv::Mat &frame)
{
    auto blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(), true,false);
    net.setInput(blob);
    vector<cv::Mat> outs;
    net.forward(outs, get_outputs_names(net));

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);

            if (confidence > confThreshold and classes[classIdPoint.x] == "person")
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    std::vector<cv::Rect> new_boxes;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        new_boxes.push_back(box);
        //drawPred(classIds[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, frame);
    }
    return new_boxes;
}
std::vector<cv::String> SpecificWorker::get_outputs_names(const cv::dnn::Net &net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<cv::String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}
void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer->scene.removeItem(laser_polygon);

    QPolygonF poly;
    poly << QPointF(0,0);
    for(auto &&l : ldata)
        poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}
Eigen::Vector2f SpecificWorker::from_robot_to_world(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(r_state.rz) , -sin(r_state.rz) , sin(r_state.rz) , cos(r_state.rz);
    return (matrix * p) + Eigen::Vector2f(r_state.x, r_state.y);
}
Eigen::Vector2f SpecificWorker::from_world_to_robot(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(r_state.rz) , -sin(r_state.rz) , sin(r_state.rz) , cos(r_state.rz);
    return (matrix.transpose() * (p - Eigen::Vector2f(r_state.x, r_state.y)));
}
QPointF SpecificWorker::toQPointF(const Eigen::Vector2f &p)
{ return QPointF(p.x(),p.y());};
Eigen::Vector2f SpecificWorker::toEigen2f(const QPointF &p)
{ return Eigen::Vector2f(p.x(),p.y());};

////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompEmotionalMotor you can call this methods:
// this->emotionalmotor_proxy->expressAnger(...)
// this->emotionalmotor_proxy->expressDisgust(...)
// this->emotionalmotor_proxy->expressFear(...)
// this->emotionalmotor_proxy->expressJoy(...)
// this->emotionalmotor_proxy->expressSadness(...)
// this->emotionalmotor_proxy->expressSurprise(...)




/////// HOG

// body
// vector<cv::Rect> found;
// vector<double> weights;
// cv::Mat n_img_bw;
// cv::cvtColor(n_img, n_img_bw, cv::COLOR_BGR2GRAY);
// hog.detectMultiScale(n_img_bw, found, weights);


/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompEmotionalMotor you can call this methods:
// this->emotionalmotor_proxy->expressAnger(...)
// this->emotionalmotor_proxy->expressDisgust(...)
// this->emotionalmotor_proxy->expressFear(...)
// this->emotionalmotor_proxy->expressJoy(...)
// this->emotionalmotor_proxy->expressSadness(...)
// this->emotionalmotor_proxy->expressSurprise(...)

/**************************************/
// From the RoboCompJointMotorSimple you can call this methods:
// this->jointmotorsimple_proxy->getMotorParams(...)
// this->jointmotorsimple_proxy->getMotorState(...)
// this->jointmotorsimple_proxy->setPosition(...)
// this->jointmotorsimple_proxy->setVelocity(...)
// this->jointmotorsimple_proxy->setZeroPos(...)

/**************************************/
// From the RoboCompJointMotorSimple you can use this types:
// RoboCompJointMotorSimple::MotorState
// RoboCompJointMotorSimple::MotorParams
// RoboCompJointMotorSimple::MotorGoalPosition
// RoboCompJointMotorSimple::MotorGoalVelocity




//                std::vector<cv::Rect> faces;
//                // Convert frame to grayscale because it requires grayscale image.
//                cv::Mat gray;
//                cv::cvtColor(n_img, gray, cv::COLOR_BGR2GRAY);
//                // Detect faces
//                faceDetector.detectMultiScale(gray, faces);
//                // Landmarks for one face is a vector of points
//                // There can be more than one face in the image. Hence, we use a vector of vector of points.
//                std::vector< vector<cv::Point2f> > landmarks;
//                // Run landmark detector
//                bool success = facemark->fit(n_img,faces,landmarks);
//                if(success)
//                    qInfo() << __FUNCTION__ << "Landmarks" << landmarks.size();